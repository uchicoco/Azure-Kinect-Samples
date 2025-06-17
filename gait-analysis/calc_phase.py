import sys
import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import hilbert


def main():
    parser = argparse.ArgumentParser(description="Calculate Hilbert transform phase from CSV and save to new CSV")
    parser.add_argument("--output", help="Output CSV file path (default: phase.csv)", default="phase.csv")
    parser.add_argument("--plot", action="store_true", help="Plot phase data")
    args = parser.parse_args()

    print(args.output)

    # File path and column names
    csv_file = "filtered_1_ANGLE_RIGHT_ANGLE_LEFT_ANGLE_LEGS.csv"
    selected_columns = ["ANGLE_RIGHT_Filtered", "ANGLE_LEFT_Filtered", "ANGLE_LEGS_Filtered"]

    # Detect BodyID
    body_ids = set()
    with open(csv_file, newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            if row and row[0]:
                body_ids.add(row[0])
    
    body_ids = sorted(list(body_ids))
    print(f"Found BodyIDs: {body_ids}")

    all_phase_data = {}
    
    for body_id in body_ids:
        print(f"Processing BodyID: {body_id}")
        
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
                if row[0] == body_id:
                    for col, idx in zip(selected_columns, col_indices):
                        values_dict[col].append(float(row[idx]))

        # Hilbert transfrom
        phase_dict = {}
        for col in selected_columns:
            values = np.array(values_dict[col])
            if len(values) > 0:
                analytic_signal = hilbert(values)
                phase = np.angle(analytic_signal)
                phase_dict[col] = phase
            else:
                phase_dict[col] = np.array([])
        
        all_phase_data[body_id] = phase_dict

    # Output CSV
    output_header = []
    for body_id in body_ids:
        for col in selected_columns:
            output_header.append(f"{body_id}_{col}_phase")

    with open(args.output, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(output_header)
        
        max_rows = 0
        for body_id in body_ids:
            for col in selected_columns:
                if len(all_phase_data[body_id][col]) > max_rows:
                    max_rows = len(all_phase_data[body_id][col])
        
        # Write data
        for i in range(max_rows):
            row_data = []
            for body_id in body_ids:
                for col in selected_columns:
                    phase_data = all_phase_data[body_id][col]
                    if i < len(phase_data):
                        row_data.append(phase_data[i])
                    else:
                        row_data.append('')
            writer.writerow(row_data)

    print(f"Phase data saved to {args.output}")

    # Plot
    if args.plot:
        fig, axes = plt.subplots(len(body_ids), 1, figsize=(12, 6*len(body_ids)))
        if len(body_ids) == 1:
            axes = [axes]
        
        for idx, body_id in enumerate(body_ids):
            ax = axes[idx]
            for col in selected_columns:
                phase_data = all_phase_data[body_id][col]
                if len(phase_data) > 0:
                    ax.plot(phase_data, label=f'{col} Phase', linewidth=2)
            
            ax.set_xlabel('Index')
            ax.set_ylabel('Phase (radians)')
            ax.set_title(f"Hilbert Transform Phase for BodyID {body_id}")
            
            ax.set_yticks([-np.pi, -np.pi/2, 0, np.pi/2, np.pi])
            ax.set_yticklabels(['-π', '-π/2', '0', 'π/2', 'π'])
            
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save and
        plot_filename = 'phase.png'
        plt.savefig(plot_filename)
        print(f"Plot saved to {plot_filename}")
        plt.show()


if __name__ == "__main__":
    main()