from bayes_opt import BayesianOptimization
import matlab.engine
import numpy as np
import matplotlib.pyplot as plt

# 启动 MATLAB 引擎
eng = matlab.engine.start_matlab()
eng.addpath(r'D:\wty study\MPC-Bayesian')  # 设置 MATLAB 路径

# 定义贝叶斯优化的目标函数，用于最小化参考值误差
# 添加平滑权重
lambda_smooth = 0.1  # 平滑惩罚项的权重，可以根据需求调整

def bayesopt_objective(**kwargs):
    ref_values = [kwargs[f'ref_value_{i}'] for i in range(1, N_ref + 1)]
    ref_values_matlab = matlab.double(ref_values)
    try:
        ref_error = eng.optimize_reference_objective(ref_values_matlab, gain, theta_const, theta_delay, nargout=1)
        # 计算平滑惩罚项
        smooth_penalty = sum([(ref_values[i] - ref_values[i-1])**2 for i in range(1, len(ref_values))])
        return -(ref_error + lambda_smooth * smooth_penalty)  # 最小化目标函数
    except matlab.engine.MatlabExecutionError as e:
        print(f"MATLAB error: {e}")
        return float('inf')


# 定义参考值范围和贝叶斯优化
N_ref = 5  # 参考值个数
gain = 3.5
theta_const = 23
theta_delay = 3

ref_bounds = {f'ref_value_{i}': (25, 35) for i in range(1, N_ref + 1)}

def bayes_opt(init_points, n_iter):
    opt = BayesianOptimization(
        f=bayesopt_objective,
        pbounds=ref_bounds,
        verbose=2
    )

    opt.maximize(
        init_points=init_points,
        n_iter=n_iter
    )

    best_ref_values = [opt.max["params"][f'ref_value_{i}'] for i in range(1, N_ref + 1)]
    print("\nBest Reference Values:", best_ref_values)
    return best_ref_values

if __name__ == "__main__":
    # Step 1: 使用贝叶斯优化计算最佳参考值
    best_ref_values = bayes_opt(2, 10)
    best_ref_values_matlab = matlab.double([float(x) for x in best_ref_values])

    # Step 2: 使用参考值进行MPC控制
    control_inputs = [0] * N_ref  # 初始化控制输入
    control_inputs_matlab = matlab.double([float(x) for x in control_inputs])

    mpc_error = eng.mpc_with_reference(
        best_ref_values_matlab,
        float(gain),  # 确保是 float 类型
        float(theta_const),
        float(theta_delay),
        control_inputs_matlab,
        nargout=1
    )

    print("MPC error with optimized reference values:", mpc_error)

    # 调用 MATLAB 函数并接收历史数据
    mpc_error, temp_history, ref_values, u_history = eng.mpc_with_reference(
        matlab.double(best_ref_values),
        float(gain),
        float(theta_const),
        float(theta_delay),
        matlab.double(control_inputs),
        nargout=4
    )



    # 转换 MATLAB 返回的历史数据为 NumPy 数组
    temp_history = np.array(temp_history).flatten()
    ref_values = np.array(ref_values).flatten()
    u_history = np.array(u_history).flatten()

    # 绘制温度跟踪曲线
    plt.figure()
    plt.plot(temp_history, label="Actual Temperature", marker='o')
    plt.plot(ref_values, label="Reference Values", linestyle='--')
    plt.xlabel("Time Step")
    plt.ylabel("Temperature (°C)")
    plt.title("Temperature Tracking")
    plt.legend()
    plt.grid()

    # 绘制控制输入曲线
    plt.figure()
    plt.plot(u_history, label="Control Inputs", marker='x')
    plt.xlabel("Time Step")
    plt.ylabel("Control Input")
    plt.title("Control Input History")
    plt.legend()
    plt.grid()

    # 显示所有图
    plt.show()

