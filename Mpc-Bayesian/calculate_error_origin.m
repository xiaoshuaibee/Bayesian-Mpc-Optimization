function [error, mpc_error] = calculate_error_origin(gain, theta_const, theta_delay, control_inputs)
    % 模型参数
    Temp_env_k = 22; % 环境温度

    % 时间设置
    Ts = 0.5; % 采样时间
    t_pred_horizon = 8; % 预测时域
    N_pred = t_pred_horizon / Ts;
    t_start = 0;
    t_stop = 300; % 总时间长度
    N_sim = (t_stop - t_start) / Ts;

    % 初始化
    Temp_out_sim_k = 28; % 初始温度
    d_sim_k = -0.5; % 干扰
    delay_array = zeros(1, floor(theta_delay / Ts) + 1);
    Temp_sp_const = 30; % 目标温度
    Temp_sp_array = ones(1, N_sim) * Temp_sp_const;

    % 初始化输出变量，防止未赋值错误
    error = Inf;
    mpc_error = Inf;

     % 贝叶斯优化误差部分
    disp('Bayesian Optimization Error Calculation Start');
    disp(['Initial Temp_out_sim_k: ', num2str(Temp_out_sim_k)]);

    try
        Temp_out_sim_plot_array = zeros(1, N_sim);
        for k = 1:(N_sim - N_pred)
            if k > 1
                max_change = 0.1 * control_inputs(min(k-1, length(control_inputs)));
                u_k = max(control_inputs(min(k, length(control_inputs))) - max_change, ...
                          min(control_inputs(min(k, length(control_inputs))) + max_change, ...
                              control_inputs(min(k, length(control_inputs)))));
            else
                u_k = control_inputs(min(k, length(control_inputs)));
        end
            delay_array = [u_k, delay_array(1:end-1)];
            Temp_out_sim_k = update_temperature_inline(Temp_out_sim_k, gain, theta_const, delay_array, d_sim_k, Ts, Temp_env_k);
            Temp_out_sim_plot_array(k) = Temp_out_sim_k;

            % 调试打印
            %disp(['Control Input u_k: ', num2str(u_k)]);
            %disp(['Temp_out_sim_k: ', num2str(Temp_out_sim_k)]);
        end
        bayesian_tracking_error = sum((Temp_out_sim_plot_array - Temp_sp_array(1:length(Temp_out_sim_plot_array))).^2);
        error = bayesian_tracking_error;
        disp(['Bayesian Tracking Error: ', num2str(bayesian_tracking_error)]);
    catch ME
        disp(['Error in Bayesian optimization part: ', ME.message]);
        error = Inf;
    end

    % MPC 误差部分
    disp('MPC Error Calculation Start');

    try
        Temp_out_sim_k = 28;
        delay_array = zeros(1, floor(theta_delay / Ts) + 1);
        Temp_out_sim_plot_array_mpc = zeros(1, N_sim);
        u_guess = double(zeros(N_pred, 1)); % 初始猜测控制信号

        for k = 1:(N_sim - N_pred)
            Temp_sp_to_mpc_array = Temp_sp_array(k:k+N_pred);
            disp(['Temp_sp_to_mpc_array: ', num2str(Temp_sp_to_mpc_array)]);

            fun_handle = @(u) mpc_objective(u, Temp_sp_to_mpc_array, Temp_out_sim_k, gain, theta_const, Ts);
            try
                u_opt = fmincon(fun_handle, u_guess, [], [], [], [], zeros(N_pred, 1), ones(N_pred, 1) * 5, [], fmincon_options);
            catch ME
                disp(['Error in fmincon: ', ME.message]);
                mpc_error = Inf;
                return;
            end
            u_k = u_opt(1);
            delay_array = [u_k, delay_array(1:end-1)];
            Temp_out_sim_k = update_temperature_inline(Temp_out_sim_k, gain, theta_const, delay_array, d_sim_k, Ts, Temp_env_k);
            Temp_out_sim_plot_array_mpc(k) = Temp_out_sim_k;

            % 调试打印
            disp(['Optimized Control u_opt: ', num2str(u_opt')]);
            disp(['Applied Control u_k: ', num2str(u_k)]);
            disp(['Temp_out_sim_k after MPC step: ', num2str(Temp_out_sim_k)]);

            % 更新初始猜测
            u_guess = [u_opt(2:end); u_opt(end)];
        end
        mpc_tracking_error = sum((Temp_out_sim_plot_array_mpc - Temp_sp_array(1:length(Temp_out_sim_plot_array_mpc))).^2);
        mpc_error = mpc_tracking_error;
        disp(['MPC Tracking Error: ', num2str(mpc_tracking_error)]);
    catch ME
        disp(['Error in MPC control part: ', ME.message]);
        mpc_error = Inf;
    end

    % 返回前打印最终误差
    disp(['Final Bayesian Error: ', num2str(error)]);
    disp(['Final MPC Error: ', num2str(mpc_error)]);

end

    function cost = mpc_objective(u, Temp_sp_array, Temp_out_sim_k, gain, theta_const, Ts)
        N_pred = length(u);
        Temp_out_sim_k_next = Temp_out_sim_k;
        cost = 0;
        for i = 1:N_pred
            Temp_out_sim_k_next = Temp_out_sim_k_next + Ts * ((gain * u(i)) - theta_const * (Temp_out_sim_k_next - Temp_sp_array(i)));
            normalized_error = (Temp_out_sim_k_next - Temp_sp_array(i)) / max(abs(Temp_sp_array(i)), 1e-6);
            cost = cost + normalized_error^2;
        end
        cost = double(cost); % 强制返回值为 double
    end

    function Temp_out_sim_k = update_temperature_inline(Temp_out_sim_k, gain, theta_const, delay_array, d_sim_k, Ts, Temp_env_k)
    Temp_out_sim_k = Temp_out_sim_k + Ts * ((gain * delay_array(1)) - theta_const * (Temp_out_sim_k - Temp_env_k) + d_sim_k);
end
