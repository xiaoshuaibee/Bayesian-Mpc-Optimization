function [mpc_error, temp_history, ref_values, u_history] = mpc_with_reference(ref_values, gain, theta_const, theta_delay, control_inputs)
    % 初始化
    Ts = 0.5; % 采样时间
    N_ref = length(ref_values);
    Temp_out_sim_k = 10; % 初始温度
    delay_array = zeros(1, floor(theta_delay / Ts) + 1);
    Temp_out_sim_plot_array = zeros(1, N_ref);
    u_history = zeros(1, N_ref); % 记录控制输入

    % 初始猜测值
    u_guess = zeros(N_ref, 1);  % 全零向量作为初始值
    lb = zeros(N_ref, 1);       % 控制输入的下界
    ub = ones(N_ref, 1) * 5;    % 控制输入的上界
    options = optimoptions('fmincon', 'Display', 'off');

    % 动态优化跟踪参考值
    for k = 1:(N_ref - 1)
        y_ref_k = ref_values(k); % 当前参考值
        fun_handle = @(u) mpc_objective(u, y_ref_k, Temp_out_sim_k, gain, theta_const, Ts);

        % 使用 fmincon 优化
        u_opt = fmincon(fun_handle, u_guess, [], [], [], [], lb, ub, [], options);
        u_k = u_opt(1); % 当前控制输入
        delay_array = [u_k, delay_array(1:end-1)];
        Temp_out_sim_k = update_temperature_inline(Temp_out_sim_k, gain, theta_const, delay_array, 0, Ts, 25);

        % 记录输出和控制输入
        Temp_out_sim_plot_array(k) = Temp_out_sim_k;
        u_history(k) = u_k;

        % 更新下一步的初始猜测
        u_guess = [u_opt(2:end); u_opt(end)];
    end

    % 计算跟踪误差
    mpc_error = sum((Temp_out_sim_plot_array - ref_values).^2);
    temp_history = Temp_out_sim_plot_array; % 输出温度历史
end


function cost = mpc_objective(u, y_ref_k, Temp_out_sim_k, gain, theta_const, Ts)
    cost = 0;
    for i = 1:length(u)
        Temp_out_sim_k = Temp_out_sim_k + Ts * ((gain * u(i)) - theta_const * (Temp_out_sim_k - y_ref_k));
        cost = cost + (Temp_out_sim_k - y_ref_k)^2; % 跟踪误差
        if i > 1
            cost = cost + 0.1 * (u(i) - u(i-1))^2; % 控制输入变化惩罚
        end
    end
    cost = double(cost);
end