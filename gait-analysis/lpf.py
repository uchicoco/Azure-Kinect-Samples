import csv
import sys
import numpy as np
import argparse
import matplotlib.pyplot as plt

def lpf(fs, fc, x):
    omega_c = 2*np.pi*(fc/fs)
    alpha = np.exp(-omega_c)

    y = np.zeros_like(x)
    # y[0] = (1-alpha)*x[0] # y[-1] = 0
    y[0] = x[0] # y[-1] = x[0]
    for n in range(1, len(x)):
        y[n] = (1-alpha)*x[n] + alpha*y[n-1]
    return y

def main():
    fs = 30
    fc = 3.3

    parser = argparse.ArgumentParser(description="Plot selected columns from CSV where first column matches a value.")
    parser.add_argument("csv_file", help="CSV file path")
    parser.add_argument("target_number", help="Value to match in the first column")
    parser.add_argument("--columns", required=True, help="Comma-separated column names to plot")
    parser.add_argument("--save-csv", action="store_true", help="Save filtered values to CSV")
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

    # フィルタリング処理と結果保存
    filtered_dict = {}
    plt.figure()
    for col in selected_columns:
        values = np.array(values_dict[col])
        plt.plot(values, label=f'{col} Original', linestyle='--')
        filtered_values = lpf(fs, fc, values)
        filtered_dict[col] = filtered_values
        plt.plot(filtered_values, label=f'{col} LPF')

    # CSVに保存
    if args.save_csv:
        output_csv = f'filtered_{target_number}_{"_".join(selected_columns)}.csv'
        with open(output_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            # ヘッダー書き込み（元CSVの最初の列名を取得）
            first_col_name = header[0]  # 元CSVの最初の列名
            header_row = [first_col_name, 'Index'] + [f'{col}_Original' for col in selected_columns] + [f'{col}_Filtered' for col in selected_columns]
            writer.writerow(header_row)
            
            # データ書き込み
            max_len = max(len(values_dict[col]) for col in selected_columns)
            for i in range(max_len):
                row = [target_number, i]  # 最初の列にtarget_numberを保持
                # オリジナル値
                for col in selected_columns:
                    if i < len(values_dict[col]):
                        row.append(values_dict[col][i])
                    else:
                        row.append('')
                # フィルタリング済み値
                for col in selected_columns:
                    if i < len(filtered_dict[col]):
                        row.append(filtered_dict[col][i])
                    else:
                        row.append('')
                writer.writerow(row)
        print(f"Filtered data saved to {output_csv}")

    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.title(f"Filtered value of {selected_columns[0]} (ID: {target_number})")
    plt.legend()
    plt.savefig(f'filtered_{target_number}_{"_".join(selected_columns)}.png')

if __name__ == "__main__":
    main()
