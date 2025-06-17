import sys
import csv
import argparse
import matplotlib.pyplot as plt
from scipy.signal import hilbert
import numpy as np


def main():
    parser = argparse.ArgumentParser(description="Plot selected columns from CSV where first column matches a value.")
    parser.add_argument("csv_file", help="CSV file path")
    parser.add_argument("target_number", help="Value to match in the first column")
    parser.add_argument("--columns", required=True, help="Comma-separated column names to plot")
    parser.add_argument("--hilbert", action="store_true", help="Also plot Hilbert transform phase")
    args = parser.parse_args()

    csv_file = args.csv_file
    target_number = args.target_number
    selected_columns = [col.strip() for col in args.columns.split(",")]

    with open(csv_file, newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        col_indices = []
        for col in selected_columns:
            if col not in header:
                print(f"Column '{col}' not found in CSV header.")
                sys.exit(1)
            col_indices.append(header.index(col))

        values_dict = {col: [] for col in selected_columns}
        for row in reader:
            if not row:
                continue
            if row[0] == target_number:
                for col, idx in zip(selected_columns, col_indices):
                    values_dict[col].append(float(row[idx]))

    # Plot
    plt.figure(figsize=(12, 8))
    for col in selected_columns:
        values = np.array(values_dict[col])
        plt.plot(values, label=f'{col} Original')
        if args.hilbert:
            analytic_signal = hilbert(values)
            envelope = np.abs(analytic_signal)
            phase = np.angle(analytic_signal)
            plt.plot(phase, label=f'{col} Hilbert Phase', linestyle='-')
            plt.plot(envelope, label=f'{col} Hilbert Envelope', alpha=0.2)

    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.title(f"Selected columns for first column == {target_number}")
    plt.legend()
    plt.savefig(f'plot_{target_number}_{"_".join(selected_columns)}.png')

    # Plot phase
    if args.hilbert:
        plt.figure(figsize=(12, 6))
        for col in selected_columns:
            values = np.array(values_dict[col])
            analytic_signal = hilbert(values)
            phase = np.angle(analytic_signal)
            plt.plot(phase, label=f'{col} Hilbert Phase', linewidth=2)

        plt.xlabel('Index')
        plt.ylabel('Phase (radians)')
        plt.title(f"Hilbert Transform Phase for first column == {target_number}")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.savefig(f'phase_{target_number}_{"_".join(selected_columns)}.png')


if __name__ == "__main__":
    main()
