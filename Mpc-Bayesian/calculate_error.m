function error = calculate_error(gain, theta_const, theta_delay)
    % 模型参数
    model_params.gain = gain;
    model_params.theta_const = theta_const;
    model_params.theta_delay = theta_delay;
    Temp_env_k = 25;

    % 时间设置
    Ts = 0.5;
    t_pred_horizon = 8;
    N_pred = t_pred_horizon / Ts;
    t_start = 0;
    t_stop = 300;
    N_sim = (t_stop - t_start) / Ts;
    t = [t_start:Ts:t_stop-Ts];

    % MPC 成本
    C_e = 1;
    C_du = 20;
    mpc_costs.C_e = C_e;
    mpc_costs.C_du = C_du;

    % 设定点序列
    Temp_sp_const = 30;
    Ampl_step = 2;
    Slope = -0.04;
    Ampl_sine = 1;
    T_period = 50;
    t_const_start = t_start; t_const_stop = 100;
    t_step_start = t_const_stop; t_step_stop = 150;
    t_ramp_start = t_step_stop; t_ramp_stop = 200;
    t_sine_start = t_ramp_stop; t_sine_stop = 250;
    t_const2_start = t_sine_stop; t_const2_stop = t_stop;
    
    Temp_sp_array = zeros(1, N_sim);
    for k = 1:N_sim
        if (t(k) >= t_const_start && t(k) < t_const_stop)
            Temp_sp_array(k) = Temp_sp_const;
        elseif (t(k) >= t_step_start && t(k) < t_step_stop)
            Temp_sp_array(k) = Temp_sp_const + Ampl_step;
        elseif (t(k) >= t_ramp_start && t(k) < t_ramp_stop)
            Temp_sp_array(k) = Temp_sp_const + Ampl_step + Slope * (t(k) - t_ramp_start);
        elseif (t(k) >= t_sine_start && t(k) < t_sine_stop)
            Temp_sp_array(k) = Temp_sp_const + Ampl_sine * sin(2 * pi * (1 / T_period) * (t(k) - t_sine_start));
        else
            Temp_sp_array(k) = Temp_sp_const;
        end
    end

    % 初始化
    u_init = 0;
    N_delay = floor(theta_delay / Ts) + 1;
    delay_array = zeros(1, N_delay) + u_init;

    % 初始猜测的最优控制序列
    Temp_heat_sim_k = 0;
    Temp_out_sim_k = 28;
    d_sim_k = -0.5;

    % 估计器的初始值
    Temp_heat_est_k = 0;
    Temp_out_est_k = 25;
    d_est_k = 0;

    % 初始猜测的最优控制序列
    u_guess = zeros(N_pred, 1) + u_init;

    % 之前最优值的初始值
    u_opt_km1 = u_init;

    % 定义绘图数组
    Temp_out_sim_plot_array = zeros(1, N_sim);

    % 定义线性约束矩阵
    A = [];
    B = [];
    Aeq = [];
    Beq = [];

    % 定义优化变量的上下限
    u_max = 5;
    u_min = 0;
    u_ub = zeros(1, N_pred) + u_max;
    u_lb = zeros(1, N_pred) + u_min;

    u_delayed_k = 2;

    % 计算观测器增益
    A_est = [-1 / theta_const, (gain + 0.1 * randn) / theta_const; 0, 0];
    C_est = [1 0];
    n_est = length(A_est);
    Tr_est = 5;
    T_est = Tr_est / n_est;
    estimpoly = [T_est * T_est, sqrt(2) * T_est, 1];
    eig_est = roots(estimpoly);
    K_est1 = acker(A_est', C_est', eig_est);
    K_est = K_est1';

    % MPC循环
    for k = 1:(N_sim - N_pred)
        t_k = t(k);

        % 估计输入扰动
        e_est_k = Temp_out_sim_k - Temp_out_est_k;
        dTemp_heat_est_dt_k = ...
            (1 / theta_const) * (-Temp_heat_est_k + (gain + 0.1 * randn) * (u_delayed_k + d_est_k))...
            + K_est(1) * e_est_k;
        dd_est_dt_k = 0 + K_est(2) * e_est_k;
        Temp_heat_est_kp1 = Temp_heat_est_k + Ts * dTemp_heat_est_dt_k;
        d_est_kp1 = d_est_k + Ts * dd_est_dt_k;
        Temp_out_est_k = Temp_heat_est_k + Temp_env_k;

        % 设定点数组
        Temp_sp_to_mpc_array = Temp_sp_array(k:k+N_pred);

        % 估计状态
        state_est.Temp_heat_est_k = Temp_heat_est_k;
        state_est.d_est_k = d_est_k;

        % 计算最优控制序列
        fun_handle = @(u) fun_objectfunction_mpc_airheater...
            (u, u_opt_km1, state_est, Temp_env_k, Temp_sp_to_mpc_array, model_params, mpc_costs, N_pred, Ts);

        fmincon_options = optimoptions(@fmincon, 'display', 'none');
        [u_opt, ~, ~, ~, ~, ~, ~] = fmincon(fun_handle, u_guess, A, B, Aeq, Beq, u_lb, u_ub, @fun_constraints_mpc_airheater, fmincon_options);

        u_guess = u_opt;
        u_k = u_opt(1);
        u_opt_km1 = u_opt(1);

        % 应用最优控制信号
        d_sim_k = -0.5;
        u_delayed_k = delay_array(N_delay);
        u_nondelayed_k = u_k;
        delay_array = [u_nondelayed_k, delay_array(1:end-1)];
        dTemp_heat_sim_dt_k = ...
            (1 / theta_const) * (-Temp_heat_sim_k + (gain + 0.1 * randn) * (u_delayed_k + d_sim_k));
        Temp_heat_sim_kp1 = Temp_heat_sim_k + Ts * dTemp_heat_sim_dt_k;
        Temp_out_sim_k = Temp_heat_sim_k + Temp_env_k;
        Temp_out_sim_plot_array(k) = Temp_out_sim_k;

        % 时间偏移
        Temp_heat_est_k = Temp_heat_est_kp1;
        d_est_k = d_est_kp1;
        Temp_heat_sim_k = Temp_heat_sim_kp1;
    end

    % 定义控制误差
    error = sum((Temp_out_sim_plot_array - Temp_sp_array(1:length(Temp_out_sim_plot_array))).^2);
end
