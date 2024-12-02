import matlab.engine

# 启动 MATLAB 引擎
eng = matlab.engine.start_matlab()

# 添加 calculate_error.m 文件所在路径
eng.addpath(r'D:\wty study\MPC-Bayesian')

# 调用 calculate_error 函数
try:
    result = eng.calculate_error(1.0, 20.0, 5.0)
    print("Result from MATLAB:", result)
except Exception as e:
    print("Error:", e)

# 关闭 MATLAB 引擎
eng.quit()