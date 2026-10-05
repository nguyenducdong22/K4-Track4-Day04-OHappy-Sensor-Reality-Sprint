"""
T6 - Drone rangefinder fault detector (OHappy, K4 Track4 Day04)
Benchmark mo phong closed-loop: drone ha canh tu 10 m bang rangefinder (ToF/LiDAR 1 tia).

So sanh 5 cach xu ly rangefinder tren CUNG chuoi du lieu / cung seed:
  raw         : tin thang cam bien (timeout -> giu gia tri cu)
  median      : validity check (min/max, timeout) + median filter cua so 5 mau
  mad         : validity check + Hampel / MAD z-score (cua so 15, |z| > 3)
  kf          : validity check + Kalman 1D (du doan bang IMU) + innovation gate
                + kinematic consistency (stuck) + baro consistency  (y tuong tu PX4 EKF2)
  kf_fallback : kf + luat fallback S1/S2/S3 do nhom dinh nghia (de xuat cai tien)

Day la ket qua benchmark TU CHAY cua nhom tren du lieu tong hop, KHONG phai so lieu cua PX4.

Chay:   python rangefinder_benchmark.py          (khoang 30-60 s)
Output: out/benchmark_log.txt, out/results.csv, out/results_raw.csv, out/*.png
"""
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "out")
os.makedirs(OUT, exist_ok=True)

# ------------------------------------------------------------------ cau hinh
DT = 0.05             # 20 Hz rangefinder
H0 = 10.0             # do cao bat dau (m)
R_MIN, R_MAX = 0.1, 12.0
T_MAX = 60.0
N_SEEDS = 40
V_FAST, V_SLOW, H_SLOW = 1.5, 0.3, 2.5   # ha 1.5 m/s; khi uoc luong < 2.5 m thi ha 0.3 m/s

# muc loi -> ten theo tham so thuc
FAULTS = {
    "clean":        [(0.0,  "baseline")],
    "noise":        [(0.10, "sigma 0.1 m"), (0.30, "sigma 0.3 m"), (0.60, "sigma 0.6 m")],
    "dropout":      [(0.10, "timeout 10%"), (0.30, "timeout 30%"), (0.60, "timeout 60%")],
    "out_of_range": [(0.05, "invalid 5%"),  (0.15, "invalid 15%"), (0.30, "invalid 30%")],
    "spike":        [(0.02, "spike 2%"),    (0.05, "spike 5%"),    (0.10, "spike 10%")],
    "stuck":        [(1.0,  "stuck 1 s"),   (3.0,  "stuck 3 s"),   (99.0, "stuck den het")],
    "bias":         [(0.3,  "bias +0.3 m"), (0.8,  "bias +0.8 m"), (1.5,  "bias +1.5 m")],
    "terrain":      [(0.5,  "step 0.5 m"),  (1.0,  "step 1.0 m"),  (2.0,  "step 2.0 m")],  # KHONG phai loi
}
MODES = ["raw", "median", "mad", "kf", "kf_fallback"]
COLORS = {"raw": "#9aa0a6", "median": "#e8a33d", "mad": "#8e44ad", "kf": "#2b6cb0", "kf_fallback": "#1b8a5a"}

# luat fallback S1/S2/S3 (nhom tu dinh nghia) - cua so 1 s = 20 mau
S2_REJECT = 0.10      # >=10% mau bi gat trong 1 s  -> S2 degraded
S3_REJECT = 0.40      # >=40% mau bi gat trong 1 s  -> S3 failed
S3_TIMEOUT_S = 0.5    # mat/invalid lien tuc >= 0.5 s -> S3 failed
V_S2 = 0.5            # S2: gioi han toc do ha 0.5 m/s
V_S3 = 0.3            # S3: bo rangefinder, dung baro+IMU, ha 0.3 m/s


# ------------------------------------------------------------------ cam bien
class RangeFinder:
    def __init__(self, rng, fault, level):
        self.rng, self.fault, self.level = rng, fault, level
        self.t_fault = rng.uniform(3.0, 7.0)      # thoi diem loi bat dau (theo seed)
        self.dropped = False
        self.stuck_val = None

    def read(self, t, h):
        """tra ve (gia tri do | None neu timeout, nhan loi that 0/1)"""
        rng = self.rng
        r = h + rng.normal(0, 0.02 + 0.005 * h)    # nhieu nen: 2 cm + 0.5% khoang cach
        if h > R_MAX:
            return None, 0
        if self.fault in ("clean", "terrain") or t < self.t_fault:
            return r, 0
        f, L = self.fault, self.level
        if f == "noise":                            # nang chieu vao ToF -> nhieu tang
            return r + rng.normal(0, L), 1
        if f == "dropout":                          # mat goi / timeout, theo burst (Markov)
            p_stay = 0.8
            p_enter = L * (1 - p_stay) / (1 - L)
            self.dropped = rng.random() < (p_stay if self.dropped else p_enter)
            return (None, 1) if self.dropped else (r, 0)
        if f == "out_of_range":                     # bao gia tri ngoai dai (0 hoac > max)
            if rng.random() < L:
                return (0.0 if rng.random() < 0.5 else R_MAX + 0.5), 1
            return r, 0
        if f == "spike":                            # phan xa nham (mep vat, co, bui)
            if rng.random() < L:
                return max(R_MIN, r + rng.choice([-1, 1]) * rng.uniform(1.0, 6.0)), 1
            return r, 0
        if f == "stuck":                            # driver/bus treo, lap lai gia tri cu
            if t < self.t_fault + L:
                if self.stuck_val is None:
                    self.stuck_val = r
                return self.stuck_val, 1
            return r, 0
        if f == "bias":                             # mat nuoc / co cao: doc xa hon that
            return r + L, 1
        return r, 0


def valid(r):
    return r is not None and R_MIN <= r <= R_MAX


# ------------------------------------------------------------------ cac bo xu ly
class Estimator:
    def __init__(self, mode, h0):
        self.mode = mode
        self.h_est, self.P = h0, 0.01
        self.buf = []
        self.hist = []
        self.bias_cnt = 0

    def step(self, r, baro, v_imu):
        """cap nhat h_est; tra ve (flag 1 = mau bi coi la loi, ly do)"""
        m = self.mode
        if m == "raw":
            if r is None:
                return 1, "timeout"
            self.h_est = r
            return 0, "ok"

        if m in ("median", "mad"):
            if not valid(r):
                return 1, "invalid"
            self.buf = (self.buf + [r])[-(5 if m == "median" else 15):]
            med = float(np.median(self.buf))
            if m == "median":
                self.h_est = med
                return (1, "median_dev") if abs(r - med) > 0.5 else (0, "ok")
            mad = float(np.median(np.abs(np.array(self.buf) - med))) * 1.4826
            z = abs(r - med) / max(mad, 0.03)
            if z > 3.0 and len(self.buf) >= 5:
                self.h_est = med                     # Hampel: thay outlier bang median
                return 1, "mad_z"
            self.h_est = r
            return 0, "ok"

        # ---- kf / kf_fallback
        h_pred = self.h_est + v_imu * DT
        P_pred = self.P + 0.0005
        ok, why = valid(r), ("ok" if valid(r) else "invalid")
        if ok:
            sigma = 0.02 + 0.005 * r
            # 1) kinematic consistency: range dung yen 0.5 s trong khi IMU bao dang di chuyen
            self.hist = (self.hist + [r])[-10:]
            if len(self.hist) == 10 and np.std(self.hist) < 1e-3 and abs(v_imu) > 0.15:
                ok, why = False, "stuck"
            # 2) consistency voi barometer (loi cham, kieu mat nuoc)
            self.bias_cnt = min(self.bias_cnt + 1, 40) if abs(r - baro) > 1.0 else max(self.bias_cnt - 2, 0)
            if ok and self.bias_cnt > 20:
                ok, why = False, "baro_mismatch"
            # 3) innovation gate ~3 sigma (giong EKF2_RNG_GATE)
            if ok and abs(r - h_pred) > 3.0 * np.sqrt(sigma**2 + P_pred) + 0.15:
                ok, why = False, "innovation"
        if ok:
            Rm = (0.02 + 0.005 * r) ** 2
            K = P_pred / (P_pred + Rm)
            self.h_est = h_pred + K * (r - h_pred)
            self.P = (1 - K) * P_pred
            return 0, "ok"
        self.h_est = h_pred + 0.02 * (baro - h_pred)   # dead-reckoning IMU + keo nhe ve baro
        self.P = P_pred
        return 1, why


# ------------------------------------------------------------------ mo phong ha canh
def simulate(fault, level, mode, seed, record=False):
    rng = np.random.default_rng(seed)
    sensor = RangeFinder(rng, fault, level)
    est = Estimator(mode, H0)
    noise_rng = np.random.default_rng(seed + 99991)   # IMU / baro
    h, v, t, terrain = H0, 0.0, 0.0, 0.0
    baro_bias = noise_rng.uniform(-0.4, 0.4)
    flags_win, bad_run = [], 0.0
    y_true, y_flag, resid, sq_err = [], [], [], []
    n_timeout = n_total = 0
    sev_count = {"S1": 0, "S2": 0, "S3": 0}
    log = []
    while t < T_MAX:
        if fault == "terrain" and t >= sensor.t_fault and terrain == 0.0:
            terrain = level; h -= level              # mat dat nhay len that su
        v_imu = v + noise_rng.normal(0, 0.05)
        baro = h + terrain + baro_bias + noise_rng.normal(0, 0.25)   # baro do cao tuyet doi
        r, is_fault = sensor.read(t, h)
        flag, why = est.step(r, baro, v_imu)

        # ---- sensor health metric (doc lap voi bo loc)
        n_total += 1
        if r is None:
            n_timeout += 1
        elif valid(r):
            resid.append(r - h)
        if (r is not None or is_fault) and h > 0.15:   # sat dat < R_MIN la binh thuong
            y_true.append(is_fault); y_flag.append(flag)
        if h < 3.0:
            sq_err.append((est.h_est - h) ** 2)

        # ---- muc suc khoe S1/S2/S3
        flags_win = (flags_win + [flag])[-20:]
        bad_run = bad_run + DT if not valid(r) else 0.0
        q = np.mean(flags_win)
        if why == "stuck" or q >= S3_REJECT or bad_run >= S3_TIMEOUT_S:
            sev = "S3"
        elif q >= S2_REJECT:
            sev = "S2"
        else:
            sev = "S1"
        sev_count[sev] += 1

        # ---- bo dieu khien ha canh
        v_cmd = -V_FAST if est.h_est > H_SLOW else -V_SLOW
        if mode == "kf_fallback":
            if sev == "S2":
                v_cmd = max(v_cmd, -V_S2)
            elif sev == "S3":
                v_cmd = max(v_cmd, -V_S3)
        if t < 2.0:
            v_cmd = 0.0
        v += (v_cmd - v) * DT / 0.4
        h += v * DT
        if record:
            log.append(dict(t=t, h=h, h_est=est.h_est, r=np.nan if r is None else r,
                            fault=is_fault, flag=flag, why=why, sev=sev))
        t += DT
        if h <= 0:
            break

    y_true, y_flag = np.array(y_true), np.array(y_flag)
    res = dict(fault=fault, level=level, mode=mode, seed=seed, t_land=t, v_td=abs(v),
               tp=int(np.sum((y_true == 1) & (y_flag == 1))), fp=int(np.sum((y_true == 0) & (y_flag == 1))),
               fn=int(np.sum((y_true == 1) & (y_flag == 0))), tn=int(np.sum((y_true == 0) & (y_flag == 0))),
               timeout_rate=n_timeout / n_total, range_var=float(np.var(resid)) if resid else np.nan,
               rmse_low=float(np.sqrt(np.mean(sq_err))) if sq_err else np.nan,
               s3_frac=sev_count["S3"] / n_total)
    return (res, pd.DataFrame(log)) if record else res


def risk_score(v_td):
    """Landing risk score (nhom dinh nghia): 0 khi cham dat <= 0.4 m/s, 1 khi >= 1.2 m/s"""
    return float(np.clip((v_td - 0.4) / 0.8, 0, 1))


# ------------------------------------------------------------------ chay benchmark
def main():
    rows = []
    for fault, levels in FAULTS.items():
        for li, (L, name) in enumerate(levels):
            for mode in MODES:
                for s in range(N_SEEDS):
                    r = simulate(fault, L, mode, seed=1000 * li + s)
                    r.update(level_idx=li, level_name=name)
                    rows.append(r)
    df = pd.DataFrame(rows)
    df["risk"] = df.v_td.apply(risk_score)
    df["hard"] = (df.v_td >= 0.9).astype(int)
    df.to_csv(os.path.join(OUT, "results_raw.csv"), index=False)

    g = df.groupby(["fault", "level_idx", "level_name", "mode"], sort=False).agg(
        tp=("tp", "sum"), fp=("fp", "sum"), fn=("fn", "sum"), tn=("tn", "sum"),
        timeout_rate=("timeout_rate", "mean"), range_var=("range_var", "mean"),
        rmse_low=("rmse_low", "mean"), v_td=("v_td", "mean"), risk=("risk", "mean"),
        hard_rate=("hard", "mean"), t_land=("t_land", "mean"), s3_frac=("s3_frac", "mean")).reset_index()
    g["precision"] = g.tp / (g.tp + g.fp).replace(0, np.nan)
    g["recall"] = g.tp / (g.tp + g.fn).replace(0, np.nan)
    g["false_alarm"] = g.fp / (g.fp + g.tn).replace(0, np.nan)
    g.to_csv(os.path.join(OUT, "results.csv"), index=False)

    # ---------------- log
    f2 = lambda v: "   - " if pd.isna(v) else f"{v:5.2f}"
    L = [f"T6 rangefinder fault benchmark - OHappy | du lieu tong hop, {N_SEEDS} seed/cau hinh, 20 Hz, ha canh tu {H0} m",
         "Landing risk = clip((v_cham_dat - 0.4)/0.8, 0, 1); hard landing = v_cham_dat >= 0.9 m/s",
         "tmo% = timeout rate; rvar = phuong sai (r - h_that) [m^2]; rmse = RMSE uoc luong khi h < 3 m [m]",
         "prec/rec = phat hien mau loi; FA = false alarm tren mau sach; t_land = thoi gian ha canh [s]", ""]
    hdr = (f"{'fault':12s} {'level':15s} {'mode':11s} {'tmo%':>5s} {'rvar':>6s} {'prec':>5s} {'rec':>5s} "
           f"{'FA':>5s} {'rmse':>5s} {'v_td':>5s} {'risk':>5s} {'hard%':>5s} {'t_land':>6s}")
    L += [hdr, "-" * len(hdr)]
    for _, x in g.iterrows():
        L.append(f"{x.fault:12s} {x.level_name:15s} {x['mode']:11s} {100*x.timeout_rate:5.1f} {x.range_var:6.3f} "
                 f"{f2(x.precision)} {f2(x.recall)} {f2(x.false_alarm)} {f2(x.rmse_low)} {x.v_td:5.2f} "
                 f"{x.risk:5.2f} {100*x.hard_rate:5.0f} {x.t_land:6.1f}")
    real = df[df.fault != "terrain"]
    summ = real.groupby("mode").agg(risk=("risk", "mean"), hard_rate=("hard", "mean"),
                                    v_td=("v_td", "mean"), t_land=("t_land", "mean"))
    L += ["", "TONG HOP tren baseline + tat ca loi (khong tinh terrain):", summ.reindex(MODES).round(3).to_string()]
    txt = "\n".join(L)
    open(os.path.join(OUT, "benchmark_log.txt"), "w").write(txt)
    print(txt)

    faults = ["noise", "dropout", "out_of_range", "spike", "stuck", "bias"]

    # ---------------- plot 1: raw vs filtered
    fig, axes = plt.subplots(2, 3, figsize=(16, 7.5), sharey=True)
    for ax, f in zip(axes.ravel(), faults):
        L_, name = FAULTS[f][1]
        _, lg = simulate(f, L_, "kf", seed=7, record=True)
        _, lm = simulate(f, L_, "median", seed=7, record=True)
        ax.plot(lg.t, lg.r, ".", ms=2.5, color="#bbbbbb", label="raw range")
        ax.plot(lg.t, lg.h, "k-", lw=1.5, label="do cao that")
        ax.plot(lm.t, lm.h_est, "-", color=COLORS["median"], lw=1.2, label="median filter")
        ax.plot(lg.t, lg.h_est, "-", color=COLORS["kf"], lw=1.2, label="KF + gate")
        fl = lg[lg.flag == 1]; ax.plot(fl.t, fl.r, "x", color="red", ms=3, label="KF gat bo")
        ax.set_title(f"{f}: {name}"); ax.grid(alpha=.3); ax.set_ylim(-0.5, 13)
    axes[0, 0].legend(fontsize=7); axes[1, 0].set_xlabel("t (s)"); axes[0, 0].set_ylabel("m")
    fig.suptitle("Raw vs filtered range (seed=7, muc loi giua)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "raw_vs_filtered.png"), dpi=130)

    # ---------------- plot 2: recall theo muc loi
    fig, axes = plt.subplots(1, 6, figsize=(19, 3.6), sharey=True)
    for ax, f in zip(axes, faults):
        for mode in ["median", "mad", "kf"]:
            s = g[(g.fault == f) & (g["mode"] == mode)].sort_values("level_idx")
            ax.plot(s.level_name, s.recall.fillna(0), "o-", color=COLORS[mode], label=mode)
        ax.set_title(f); ax.set_ylim(-0.05, 1.05); ax.grid(alpha=.3); ax.tick_params(axis="x", labelsize=7)
    axes[0].set_ylabel("recall phat hien loi"); axes[0].legend(fontsize=8)
    fig.suptitle("Recall phat hien mau loi theo muc loi (precision xem results.csv)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "recall_vs_level.png"), dpi=130)

    # ---------------- plot 3: landing risk muc nang
    order = ["clean"] + faults + ["terrain"]
    fig, ax = plt.subplots(figsize=(13, 4))
    w = 0.16
    for i, mode in enumerate(MODES):
        vals = [g[(g.fault == f) & (g["mode"] == mode) & (g.level_idx == (0 if f == "clean" else 2))].risk.values[0]
                for f in order]
        ax.bar(np.arange(len(order)) + (i - 2) * w, vals, w, color=COLORS[mode], label=mode)
    labels = [f"{f}\n{FAULTS[f][0 if f == 'clean' else 2][1]}" for f in order]
    ax.set_xticks(range(len(order))); ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("landing risk (0-1)"); ax.legend(ncol=5, fontsize=8); ax.grid(axis="y", alpha=.3)
    ax.set_title(f"Landing risk score o muc loi nang (trung binh {N_SEEDS} lan ha canh)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "landing_risk.png"), dpi=130)

    # ---------------- plot 4: failure case stuck
    fig, axes = plt.subplots(1, 3, figsize=(16, 4), sharey=True)
    for ax, mode in zip(axes, ["raw", "median", "kf_fallback"]):
        res, lg = simulate("stuck", 99.0, mode, seed=2002, record=True)
        ax.plot(lg.t, lg.h, "k-", lw=2, label="do cao that")
        ax.plot(lg.t, lg.r, ".", ms=3, color="#bbbbbb", label="raw range")
        ax.plot(lg.t, lg.h_est, "-", color=COLORS[mode], label="uoc luong dung cho dieu khien")
        fl = lg[lg.flag == 1]; ax.plot(fl.t, fl.r, "x", color="red", ms=4, label="bi gat bo")
        ax.set_title(f"{mode}: cham dat {res['v_td']:.2f} m/s, risk {risk_score(res['v_td']):.2f}")
        ax.set_xlabel("t (s)"); ax.grid(alpha=.3)
    axes[0].set_ylabel("m"); axes[0].legend(fontsize=8)
    fig.suptitle("Failure case: rangefinder dong bang (stuck den het) khi dang ha canh - seed 2002")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "failure_case_stuck.png"), dpi=130)

    # ---------------- plot 5: gioi han - terrain step that bi gat nham
    fig, axes = plt.subplots(1, 2, figsize=(13, 4), sharey=True)
    for ax, mode in zip(axes, ["median", "kf_fallback"]):
        res, lg = simulate("terrain", 2.0, mode, seed=2005, record=True)
        ax.plot(lg.t, lg.h, "k-", lw=2, label="do cao that (so voi mat dat)")
        ax.plot(lg.t, lg.r, ".", ms=3, color="#bbbbbb", label="raw range")
        ax.plot(lg.t, lg.h_est, "-", color=COLORS[mode], label="uoc luong")
        fl = lg[lg.flag == 1]; ax.plot(fl.t, fl.r, "x", color="red", ms=4, label="bi gat bo")
        ax.set_title(f"{mode}: cham dat {res['v_td']:.2f} m/s, t_land {res['t_land']:.1f} s")
        ax.set_xlabel("t (s)"); ax.grid(alpha=.3)
    axes[0].set_ylabel("m"); axes[0].legend(fontsize=8)
    fig.suptitle("Gioi han: mat dat that su nhay len 2 m (bay qua mai/be) - KF tuong la loi, gat bo do dung")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "limitation_terrain.png"), dpi=130)


if __name__ == "__main__":
    main()
