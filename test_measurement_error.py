"""
laser_scan_karsilastir.py

Aynı grid/ayarlarla yapılmış iki lazer tarama sonucunu (sonuclar.csv veya
xlsx) karşılaştırır ve piksel bazlı ölçüm hatasını (tekrarlanabilirlik)
raporlar.

Girdi dosyaları şu sütunları içermeli (scan scriptindeki save_csv() çıktısı):
    label, mm_x, mm_y, row, col, xu, yu, image_path

Kullanım:
    pip install pandas numpy matplotlib openpyxl
    python test_measurement_error.py tarama1.csv tarama2.csv --out sonuc
"""

import argparse
import os

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def load_scan(path):
    ext = os.path.splitext(path)[1].lower()
    df = pd.read_excel(path) if ext in (".xlsx", ".xls") else pd.read_csv(path)
    required = {"label", "mm_x", "mm_y", "row", "col", "xu", "yu"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"{path}: eksik sütun(lar): {missing}")
    return df


def compare_scans(df1, df2):
    merged = df1.merge(df2, on=["row", "col"], suffixes=("_1", "_2"),
                        how="outer", indicator=True)

    both = merged[merged["_merge"] == "both"].copy()
    nan_mask = both[["xu_1", "yu_1", "xu_2", "yu_2"]].isna().any(axis=1)
    valid = both[~nan_mask].copy()
    nan_points = both[nan_mask].copy()

    valid["dx_px"] = valid["xu_2"] - valid["xu_1"]
    valid["dy_px"] = valid["yu_2"] - valid["yu_1"]
    valid["err_px"] = np.hypot(valid["dx_px"], valid["dy_px"])

    only_1 = merged[merged["_merge"] == "left_only"]
    only_2 = merged[merged["_merge"] == "right_only"]
    return valid, nan_points, only_1, only_2


def print_report(valid, nan_points, only_1, only_2):
    print(f"Eşleşen nokta sayısı            : {len(valid) + len(nan_points)}")
    print(f"Geçerli (iki tarafta da piksel)  : {len(valid)}")
    print(f"NaN / lazer bulunamayan          : {len(nan_points)}")
    if len(only_1):
        print(f"Sadece 1. taramada olan          : {len(only_1)}")
    if len(only_2):
        print(f"Sadece 2. taramada olan          : {len(only_2)}")

    if valid.empty:
        print("Karşılaştırılacak geçerli nokta yok.")
        return

    err = valid["err_px"]
    print("\n--- PİKSEL HATASI İSTATİSTİKLERİ ---")
    print(f"Ortalama       : {err.mean():.3f} px")
    print(f"Medyan         : {err.median():.3f} px")
    print(f"Std            : {err.std():.3f} px")
    print(f"Min / Max      : {err.min():.3f} / {err.max():.3f} px")
    print(f"95. persentil  : {err.quantile(0.95):.3f} px")

    worst = valid.sort_values("err_px", ascending=False).head(10)
    print("\n--- EN KÖTÜ 10 NOKTA ---")
    cols = ["label_1", "mm_x_1", "mm_y_1", "row", "col", "dx_px", "dy_px", "err_px"]
    print(worst[cols].to_string(index=False, float_format=lambda v: f"{v:.3f}"))


def plot_heatmap(valid, out_path):
    grid = valid.pivot(index="row", columns="col", values="err_px")
    fig, ax = plt.subplots(figsize=(8, 7))
    im = ax.imshow(grid.values, cmap="inferno", aspect="auto")
    ax.set_title("Piksel hatası büyüklüğü (grid üzerinde)")
    ax.set_xlabel("col")
    ax.set_ylabel("row")
    fig.colorbar(im, ax=ax, label="hata (px)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_quiver(valid, out_path, exaggeration=20):
    fig, ax = plt.subplots(figsize=(8, 7))
    q = ax.quiver(valid["xu_1"], valid["yu_1"], valid["dx_px"], valid["dy_px"],
                   valid["err_px"], cmap="inferno", angles="xy",
                   scale_units="xy", scale=1.0 / exaggeration)
    ax.quiverkey(q, 0.85, 0.97, 1.0, f"1 px (ok {exaggeration}x büyütüldü)",
                 labelpos="E")
    ax.invert_yaxis()  # görüntü koordinatında y aşağı artar
    ax.set_title("Kayma vektörleri (1. taramadan 2. taramaya)")
    ax.set_xlabel("piksel x")
    ax.set_ylabel("piksel y")
    fig.colorbar(q, ax=ax, label="hata (px)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_error_vs_radius(valid, out_path):
    r_mm = np.hypot(valid["mm_x_1"], valid["mm_y_1"])
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.scatter(r_mm, valid["err_px"], s=14, alpha=0.7)
    ax.set_xlabel("merkeze uzaklık (mm)")
    ax.set_ylabel("piksel hatası (px)")
    ax.set_title("Hata vs. merkeze uzaklık (sistematik mi rastgele mi?)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="İki lazer tarama sonucunu karşılaştır.")
    ap.add_argument("scan1", help="1. tarama sonuç dosyası (csv/xlsx)")
    ap.add_argument("scan2", help="2. tarama sonuç dosyası (csv/xlsx)")
    ap.add_argument("--out", default="karsilastirma_sonuc", help="çıktı klasörü")
    ap.add_argument("--exaggeration", type=float, default=20,
                     help="kayma vektör grafiğinde ok büyütme katsayısı")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    df1 = load_scan(args.scan1)
    df2 = load_scan(args.scan2)
    valid, nan_points, only_1, only_2 = compare_scans(df1, df2)

    print_report(valid, nan_points, only_1, only_2)

    if not valid.empty:
        valid.to_csv(os.path.join(args.out, "detayli_karsilastirma.csv"), index=False)
        plot_heatmap(valid, os.path.join(args.out, "hata_heatmap.png"))
        plot_quiver(valid, os.path.join(args.out, "kayma_vektorleri.png"),
                    exaggeration=args.exaggeration)
        plot_error_vs_radius(valid, os.path.join(args.out, "hata_vs_uzaklik.png"))
        print(f"\nÇıktılar '{args.out}/' klasörüne kaydedildi.")


if __name__ == "__main__":
    main()