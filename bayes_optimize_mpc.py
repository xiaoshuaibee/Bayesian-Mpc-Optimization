from bayes_opt import BayesianOptimization
import matlab.engine
import numpy as np
import matplotlib.pyplot as plt

# ----------------------------------------
# 启动 MATLAB 引擎并设置路径
eng = matlab.engine.start_matlab()
eng.addpath(r'D:\wty study\MPC-Bayesian\BayesianOptimizationmaster')  # 设置你的 MATLAB 文件夹路径

# ----------------------------------------
# 定义贝叶斯优化的目标函数
def bayesopt_objective(C_e, C_du):
    try:
        cost = eng.optimize_reference_objective(float(C_e), float(C_du), nargout=1)
        return -cost  # 贝叶斯是最大化，这里取负号使之变成最小化问题
    except matlab.engine.MatlabExecutionError as e:
        print(f"MATLAB error: {e}")
        return float('inf')

# ----------------------------------------
# 设定优化变量的搜索范围
pbounds = {
    'C_e': (0.1, 10),    # 温度跟踪误差权重搜索范围
    'C_du': (0.1, 50)    # 控制输入变化量权重搜索范围
}

# ----------------------------------------
# 定义贝叶斯优化器
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

    best_params = optimizer.max['params']
    print("\nBest Parameters Found:", best_params)
    return best_params

# ----------------------------------------
# 主程序入口
if __name__ == "__main__":
    # 1. 执行贝叶斯优化，寻找最优 C_e 和 C_du
    best_params = bayes_opt(init_points=3, n_iter=15)

    # 2. 取出最优的C_e, C_du
    best_C_e = best_params['C_e']
    best_C_du = best_params['C_du']
    print(f"Best C_e: {best_C_e:.4f}, Best C_du: {best_C_du:.4f}")

    # 3. 用最优参数再仿真一遍，获取详细历史数据（可以扩展这里，比如画图）
    try:
        # cost, = eng.optimize_reference_objective(float(best_C_e), float(best_C_du), nargout=1)
        # cost = eng.optimize_reference_objective(float(best_C_e), float(best_C_du), nargout=1)
        cost = eng.optimize_reference_objective(float(best_C_e), float(best_C_du), nargout=1)
        print(f"Final Cost with Best Parameters: {cost:.4f}")
    except matlab.engine.MatlabExecutionError as e:
        print(f"MATLAB error during final simulation: {e}")



