import os
import sys
import csv
import threading
import time
import numpy as np
from scipy.signal import hilbert
from collections import deque
import argparse


class RealtimeAnglePhaseCalculator:
    def __init__(self, window_size=100, pipe_path="/tmp/angle_data_pipe"):
        self.window_size = window_size
        self.pipe_path = pipe_path
        self.selected_columns = ["right_arm_angle", "left_arm_angle", "legs_angle"]
        
        # 各BodyIDと列に対する過去データ保持用
        self.data_windows = {}
        
        # 実行制御
        self.running = False
        
        # 出力CSV初期化
        self.init_output_csv()
        
    def init_output_csv(self):
        """出力CSVファイルの初期化"""
        header = ['timestamp', 'body_id']
        for col in self.selected_columns:
            header.append(f"{col}_phase")
        
        with open("realtime_phase_output.csv", 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
    
    def add_data_point(self, body_id, timestamp, right_angle, left_angle, legs_angle):
        """新しいデータポイントを追加"""
        if body_id not in self.data_windows:
            self.data_windows[body_id] = {
                col: deque(maxlen=self.window_size) for col in self.selected_columns
            }
        
        # データポイントを各列のウィンドウに追加
        self.data_windows[body_id]["right_arm_angle"].append(float(right_angle))
        self.data_windows[body_id]["left_arm_angle"].append(float(left_angle))
        self.data_windows[body_id]["legs_angle"].append(float(legs_angle))
    
    def calculate_current_phase(self, body_id):
        """現在の位相を計算"""
        if body_id not in self.data_windows:
            return None
        
        phases = {}
        for col in self.selected_columns:
            window_data = list(self.data_windows[body_id][col])
            
            # 十分なデータがある場合のみ位相計算
            if len(window_data) >= 20:  # 最小データ数
                values = np.array(window_data)
                analytic_signal = hilbert(values)
                phase = np.angle(analytic_signal)
                # 最新の位相値を取得
                phases[col] = phase[-1]
            else:
                phases[col] = None
        
        return phases
    
    def process_angle_data(self, timestamp, body_id, right_angle, left_angle, legs_angle):
        """角度データの処理"""
        # データポイント追加
        self.add_data_point(body_id, timestamp, right_angle, left_angle, legs_angle)
        
        # 位相計算
        phases = self.calculate_current_phase(body_id)
        
        if phases and all(p is not None for p in phases.values()):
            # CSV出力
            row_data = [timestamp, body_id]
            for col in self.selected_columns:
                row_data.append(phases[col])
            
            with open("realtime_phase_output.csv", 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row_data)
            
            print(f"BodyID {body_id}: Right={phases['right_arm_angle']:.3f}, "
                  f"Left={phases['left_arm_angle']:.3f}, Legs={phases['legs_angle']:.3f}")
        
        return phases
    
    def read_from_pipe(self):
        """Named pipeからデータを読み込み"""
        print(f"Waiting for data from pipe: {self.pipe_path}")
        
        try:
            while self.running:
                # パイプが存在するかチェック
                if not os.path.exists(self.pipe_path):
                    time.sleep(0.1)
                    continue
                
                try:
                    # パイプを開いてデータを読み込み
                    with open(self.pipe_path, 'r') as pipe:
                        for line in pipe:
                            if not self.running:
                                break
                            
                            line = line.strip()
                            if not line:
                                continue
                            
                            try:
                                # データ解析: timestamp,body_id,right_angle,left_angle,legs_angle
                                parts = line.split(',')
                                if len(parts) == 5:
                                    timestamp = int(parts[0])
                                    body_id = int(parts[1])
                                    right_angle = float(parts[2])
                                    left_angle = float(parts[3])
                                    legs_angle = float(parts[4])
                                    
                                    # リアルタイム処理
                                    self.process_angle_data(timestamp, body_id, right_angle, left_angle, legs_angle)
                                    
                            except (ValueError, IndexError) as e:
                                print(f"Error parsing line: {line}, Error: {e}")
                                continue
                                
                except (OSError, IOError) as e:
                    # パイプが閉じられた場合など
                    print(f"Pipe read error: {e}")
                    time.sleep(0.1)
                    continue
                    
        except KeyboardInterrupt:
            print("\nStopping pipe reader...")
        finally:
            self.running = False
    
    def start(self):
        """リアルタイム処理開始"""
        self.running = True
        
        # パイプ読み込みスレッド開始
        pipe_thread = threading.Thread(target=self.read_from_pipe)
        pipe_thread.daemon = True
        pipe_thread.start()
        
        try:
            print("Real-time phase calculation started. Press Ctrl+C to stop.")
            while self.running:
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\nStopping real-time processing...")
            self.running = False
        
        # スレッド終了を待つ
        pipe_thread.join(timeout=2)


def main():
    parser = argparse.ArgumentParser(description="Real-time angle phase calculation from C++ data")
    parser.add_argument("--window-size", type=int, default=100, help="Window size for phase calculation")
    parser.add_argument("--pipe-path", default="/tmp/angle_data_pipe", help="Named pipe path")
    args = parser.parse_args()
    
    calculator = RealtimeAnglePhaseCalculator(
        window_size=args.window_size,
        pipe_path=args.pipe_path
    )
    
    calculator.start()


if __name__ == "__main__":
    main()