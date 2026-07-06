import os
import numpy as np
import time
import sys

# --- 关键导入 ---
# 确保你已经 'pip install ultralytics'
# FastSAMPredictor 通常在这里：

from ultralytics.models.fastsam import FastSAMPredictor


def main():
    
    try:
        
        #__file__ is the path of the current script(__file__ 是当前脚本的路径)
        script_dir = os.path.abspath(os.path.split(os.path.realpath(__file__))[0])
        # --- Focus: Model path(重点：模型路径) ---
        model_path = os.path.join(os.path.dirname(script_dir), 'resources/models', "FastSAM-x.pt")        
        # --- Key information: You are using the 'FastSAM-x.pt' (eXtra Large) model(关键信息：你使用的是 'FastSAM-x.pt' (eXtra Large)) ---
        # --- This is highly likely the reason causing the 8GB Orin memory overflow(这极有可能是导致 8GB Orin 内存溢出的原因) ---
        print("Warning: Loading the 'FastSAM-x.pt' (Extra Large) model.(警告: 正在加载 'FastSAM-x.pt' (Extra Large) 模型。)")

        overrides = dict(
            conf=0.4, 
            task="segment", 
            mode="predict", 
            model=model_path, 
            save=False, 
            imgsz=640
        )
        

        start_time = time.time()
        
        # This is the place most likely to trigger an error(这是最可能触发错误的地方)
        predictor = FastSAMPredictor(overrides=overrides) 
        
        load_time = time.time() - start_time
        print(f"...FastSAMPredictor loaded. (Time elapsed: {load_time:.2f} seconds(耗时: {load_time:.2f} 秒))")
        
        print("Warming up FastSAM model (Execute the first inference(执行第一次推理))...")
        start_time = time.time()
        
        # This is the second place that might trigger an error(这是第二个可能触发错误的地方)
        predictor(np.zeros((640, 480, 3), dtype=np.uint8)) 
        
        warmup_time = time.time() - start_time
        print(f"...FastSAM model warmed up. (耗时: {warmup_time:.2f} 秒)")
        
        print("\n" + "="*30)
        print("--- ✅ Test Successful（测试成功） ---")
        print("If this script runs successfully, it means the model itself and the environment are compatible(如果此脚本成功运行，说明模型本身和环境是兼容的。)")
        print("The issue lies with the ROS2 node or other processes consuming too much memory, leaving insufficient free memory for the model when the ROS node is running.(问题在于 ROS2 节点或其他进程占用了过多内存，导致运行 ROS 节点时没有足够剩余内存给模型。)")
        print("="*30)

    except RuntimeError as e:
        print("\n" + "="*30)
        print("--- ❌ Test Failed (Expected)(测试失败 (符合预期)) ---")
        print("Successfully reproduced the error!(成功复现了错误！)")
        print(f"Captured Runtime Error(捕获到的 Runtime Error): {e}")
        print("\n This is 99% confirmed to be insufficient memory on Jetson Orin(这 99% 确认是 Jetson Orin 内存不足。)")
        print("Reason: The 'FastSAM-x.pt' model is too large for the 8GB shared memory.(原因: 'FastSAM-x.pt' 模型对于 8GB 共享内存来说太大了。)")
        print("="*30)
    except Exception as e:
        print(f"\n--- ❌ Test Failed (Unexpected Error)(测试失败 (意外错误)) ---")
        print(f"Captured unexpected error(捕获到的意外错误): {e}")

if __name__ == "__main__":
    main()