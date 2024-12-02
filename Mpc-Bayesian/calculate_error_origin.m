function [error, baseline_error] = calculate_error2(gain, theta_const, theta_delay, control_inputs)
    % 模型参数（电采暖系统）
    model_params.gain = gain;  % 电加热的增益，表示加热功率对温度的影响
    model_params.theta_const = theta_const;  % 房间的热损失常数
    model_params.theta_delay = theta_delay;  % 控制输入的延迟时间
    Temp_env_k = 10;  % 环境温度，假设外界温度为 10 摄氏度

    % 时间设置
    Ts = 0.5;  % 采样时间，单位：小时
    t_pred_horizon = 8;  % 预测时域，单位：小时
    N_pred = t_pred_horizon / Ts;
    t_start = 0;
    t_stop = 300;  % 模拟总时长，单位：小时
    N_sim = (t_stop - t_start) / Ts;
    t = [t_start:Ts:t_stop-Ts];

    % MPC 成本
    C_e = 1;
    C_du = 20;
    mpc_costs.C_e = C_e;
    mpc_costs.C_du = C_du;

    % 初始化
    Temp_out_sim_k = 15;  % 初始的室内温度，假设为 15 摄氏度
    d_sim_k = -0.5;  % 假设有一个恒定的冷却影响（例如热损失）

    % 估计器的初始值
    Temp_heat_est_k = 0;
    Temp_out_est_k = 15;
    d_est_k = 0;

    % 初始控制信号
    N_delay = floor(theta_delay / Ts) + 1;
    delay_array = zeros(1, N_delay);

    % 设定点数组 - 初始化为恒定目标值
    Temp_sp_const = 22;  % 目标设定温度为 22 摄氏度
    Temp_sp_array = ones(1, N_sim) * Temp_sp_const;

    % 贝叶斯优化初步得到的控制信号直接用于计算误差
    Temp_out_sim_plot_array = zeros(1, N_sim);
    control_change_penalty = 0;
    control_value_penalty = 0;
    large_change_penalty = 0;
    temp_rate_of_change_penalty = 0;  % 温度变化速率的惩罚项

    try
        for k = 1:(N_sim - N_pred)
            % 设定点序列
            Temp_sp_to_mpc_array = Temp_sp_array(k:k+N_pred);

            % 估计状态
            state_est.Temp_heat_est_k = Temp_heat_est_k;
            state_est.d_est_k = d_est_k;

            % 应用控制信号
            if k > 1
                max_change = 0.1 * control_inputs(min(k-1, length(control_inputs)));
                u_k = max(control_inputs(min(k, length(control_inputs))) - max_change, min(control_inputs(min(k, length(control_inputs))) + max_change, control_inputs(min(k, length(control_inputs)))));
            else
                u_k = control_inputs(min(k, length(control_inputs)));
            end  % 选择当前控制信号
            delay_array = [u_k, delay_array(1:end-1)];
            prev_Temp_out_sim_k = Temp_out_sim_k;
            Temp_out_sim_k = update_temperature_inline(Temp_out_sim_k, gain, theta_const, delay_array, d_sim_k, Ts, Temp_env_k);
            Temp_out_sim_plot_array(k) = Temp_out_sim_k;

            % 计算控制变化率的惩罚项
            if k > 1
                change_amount = abs(u_k - control_inputs(min(k-1, length(control_inputs))));
                control_change_penalty = control_change_penalty + change_amount^2;
                % 如果变化超过某个值，增加惩罚项
                if change_amount > max_change
                    large_change_penalty = large_change_penalty + change_amount^2;
                end
            end

            % 计算控制信号绝对值的惩罚项
            control_value_penalty = control_value_penalty + u_k^2;

            % 计算温度变化速率的惩罚项
            temp_rate_of_change = abs(Temp_out_sim_k - prev_Temp_out_sim_k) / Ts;
            temp_rate_of_change_penalty = temp_rate_of_change_penalty + temp_rate_of_change^2;
        end

        % 最后误差计算（包含跟踪误差、控制变化率、控制信号绝对值、温度变化速率和过大变化的惩罚项）
        tracking_error = sum((Temp_out_sim_plot_array - Temp_sp_array(1:length(Temp_out_sim_plot_array))).^2);
        control_change_penalty_weight = 2.0;  % 控制变化率惩罚项的权重系数
        control_value_penalty_weight = 2.0;  % 控制信号绝对值惩罚项的权重系数
        large_change_penalty_weight = 5.0;  % 过大变化惩罚项的权重系数
        temp_rate_of_change_penalty_weight = 3.0;  % 温度变化速率惩罚项的权重系数
        error = tracking_error + control_change_penalty_weight * control_change_penalty + control_value_penalty_weight * control_value_penalty + large_change_penalty_weight * large_change_penalty + temp_rate_of_change_penalty_weight * temp_rate_of_change_penalty;
    catch ME
        % 如果发生错误，打印错误信息并返回一个极大值的误差
        disp(['错误: ', ME.message]);
        error = 1e6;  % 返回一个很大的误差值以表示失败
    end

    % 基准控制策略 - 恒定的输入值，作为对比
    baseline_control_input = 110;  % 假设一个恒定的输入电压值 110V
    delay_array = zeros(1, N_delay);
    Temp_out_sim_k = 15;
    Temp_out_sim_plot_array_baseline = zeros(1, N_sim);

    for k = 1:(N_sim - N_pred)
        % 恒定输入应用到系统
        u_k = baseline_control_input;
        delay_array = [u_k, delay_array(1:end-1)];
        Temp_out_sim_k = update_temperature_inline(Temp_out_sim_k, gain, theta_const, delay_array, d_sim_k, Ts, Temp_env_k);
        Temp_out_sim_plot_array_baseline(k) = Temp_out_sim_k;
    end

    % 计算基准控制策略下的误差
    tracking_error_baseline = sum((Temp_out_sim_plot_array_baseline - Temp_sp_array(1:length(Temp_out_sim_plot_array_baseline))).^2);
    baseline_error = tracking_error_baseline;
end

% 内联定义 update_temperature 函数
function Temp_out_sim_k = update_temperature_inline(Temp_out_sim_k, gain, theta_const, delay_array, d_sim_k, Ts, Temp_env_k)
    % 更新温度的逻辑 - 电采暖系统模型
    % Temp_out_sim_k: 当前的室内温度
    % gain: 电加热增益
    % theta_const: 房间的热损失常数
    % delay_array: 控制输入的延迟队列
    % d_sim_k: 外部干扰（如热损失）
    % Ts: 采样时间
    % Temp_env_k: 环境温度

    % 电采暖对室内温度的贡献，以及环境热损失的影响
    Temp_out_sim_k = Temp_out_sim_k + Ts * ((gain * delay_array(1)) - theta_const * (Temp_out_sim_k - Temp_env_k) + d_sim_k);
end
