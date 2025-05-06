from bayes_mpc_demo import BayesianOptimization
import matlab.engine
import numpy as np
import pandas as pd

# 启动 MATLAB 引擎并连接
eng = matlab.engine.start_matlab()
eng.addpath(r'D:\wty study\MPC-Bayesian')




# 定义贝叶斯优化的目标函数，用于最小化预测误差
def bayesopt_objective(**kwargs):
    # 从 kwargs 中提取所有控制输入
    control_inputs = [kwargs[f'control_input_{i}'] for i in range(1, N_pred + 1)]

    gain = 2.0
    theta_const = 3.0
    theta_delay = 1.0

    # 将控制输入转换为 MATLAB 格式
    control_inputs_matlab = matlab.double(control_inputs)

    # 调用 MATLAB 函数计算误差
    try:
        error, _ = eng.calculate_error_origin(
            gain, theta_const, theta_delay, matlab.double(control_inputs), nargout=2
        )
        if not np.isfinite(error):
            return float('inf')  # 过滤非有限值
        return error
    except matlab.engine.MatlabExecutionError as e:
        print(f"MATLAB error: {e}")
        return float('inf')  # 返回极大值以继续优化


# 定义贝叶斯优化的控制输入范围
N_pred = 5  # 假设预测时域是 5 个时间步
control_bounds = {
    f'control_input_{i}': (0, 220) for i in range(1, N_pred + 1)
}


# 贝叶斯优化的主函数
def bayes_opt(init_points, n_iter):
    opt = BayesianOptimization(
        f=bayesopt_objective,
        pbounds=control_bounds,
        verbose=2
    )

    opt.maximize(
        init_points=init_points,
        n_iter=n_iter
    )

    # 获取最佳控制输入
    best_controls = [opt.max["params"][f'control_input_{i}'] for i in range(1, N_pred + 1)]
    best_score = opt.max["target"]

    print("\n", "Best control inputs:", best_controls,
          "\n", "Best score:", best_score)

    return best_controls, best_score


if __name__ == "__main__":
    best_controls, best_score = bayes_opt(2, 10)

    gain = 3.5
    theta_const = 23
    theta_delay = 3

    # 调用 MATLAB 函数计算贝叶斯优化和 MPC 控制的误差
    control_inputs_matlab = matlab.double(best_controls)
    bayesian_error, mpc_error = eng.calculate_error_origin(
        gain, theta_const, theta_delay, control_inputs_matlab, nargout=2
    )

    # 打印对比结果
    print("\nBest score with Bayesian Optimization:", bayesian_error)
    print("MPC control error:", mpc_error)
    if bayesian_error < mpc_error:
        print("Bayesian Optimization performs better.")
    else:
        print("MPC control performs better.")