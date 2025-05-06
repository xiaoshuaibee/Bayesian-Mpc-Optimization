from bayes_opt import BayesianOptimization
import matlab.engine
import numpy as np
import pandas as pd

# ----------------------------------------
# 启动 MATLAB 引擎并设置路径
eng = matlab.engine.start_matlab()
eng.addpath(r'D:\wty study\MPC-Bayesian\BayesianOptimizationmaster')  # 替换为你的 MATLAB 文件夹路径

# ----------------------------------------
# 定义贝叶斯优化的目标函数
def bayesopt_objective(C_e, C_du):
    try:
        cost = eng.optimize_reference_objective(float(C_e), float(C_du), nargout=1)
        return -cost  # 贝叶斯优化器是最大化，这里取负使其转为最小化问题
    except matlab.engine.MatlabExecutionError as e:
        print(f"MATLAB error: {e}")
        return float('inf')

# ----------------------------------------
# 设定优化变量的搜索范围
pbounds = {
    'C_e': (0.1, 10),     # 温控误差权重
    'C_du': (0.1, 50)     # 控制输入变化惩罚
}

# ----------------------------------------
# 定义贝叶斯优化器函数
def bayes_opt(init_points=2, n_iter=10):
    optimizer = BayesianOptimization(
        f=bayesopt_objective,
        pbounds=pbounds,
        verbose=2
    )

    optimizer.maximize(
        init_points=init_points,
        n_iter=n_iter
    )

    print("\nBest Parameters Found:", optimizer.max['params'])
    return optimizer  # 返回整个优化器对象

# ----------------------------------------
# 主程序入口
if __name__ == "__main__":
    # 执行贝叶斯优化
    optimizer = bayes_opt(init_points=3, n_iter=15)

    # 提取最优参数
    best_params = optimizer.max['params']
    best_C_e = best_params['C_e']
    best_C_du = best_params['C_du']
    print(f"\nBest C_e: {best_C_e:.4f}, Best C_du: {best_C_du:.4f}")

    # 最终使用最优参数调用 MATLAB 模拟
    try:
        final_cost = eng.optimize_reference_objective(float(best_C_e), float(best_C_du), nargout=1)
        print(f"Final Cost with Best Parameters: {final_cost:.4f}")
    except matlab.engine.MatlabExecutionError as e:
        print(f"MATLAB error during final simulation: {e}")

    # 保存优化过程结果
    optimizer_results = pd.DataFrame(optimizer.res)
    optimizer_results.to_csv("optimizer_result.csv", index=False)
    print("Optimization process saved to optimizer_result.csv.")


import matplotlib.pyplot as plt

plt.plot(optimizer_results['target'], marker='o')
plt.xlabel("Iteration")
plt.ylabel("Negative Cost")
plt.title("Bayesian Optimization Process")
plt.grid(True)
plt.show()
