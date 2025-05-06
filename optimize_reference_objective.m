function cost = optimize_reference_objective(C_e, C_du)
% 优化参考目标函数，输入：C_e温度误差权重, C_du控制变化权重
% 返回：综合代价 cost

%----------------------------------
% 模型参数
gain = 0.2; %[deg C]/[V]
theta_const = 23; %[s]
theta_delay = 3; %[s]
Temp_env_k = 20; %[deg C]

model_params.gain = gain;
model_params.theta_const = theta_const;
model_params.theta_delay = theta_delay;

%----------------------------------
% 时间设置
Ts = 0.5; % 步长
t_start = 0;
t_stop = 300;
t = t_start:Ts:t_stop-Ts;
N_sim = length(t);
t_pred_horizon = 8;
N_pred = t_pred_horizon/Ts;

%----------------------------------
% MPC成本（这里传入优化参数）
mpc_costs.C_e = C_e;
mpc_costs.C_du = C_du;

%----------------------------------
% 初始设置
Temp_sp_const = 26; %[degC]
u_init = 0;
N_delay = floor(theta_delay/Ts) + 1;
delay_array = zeros(1,N_delay) + u_init;

% 初始化估计器
Temp_heat_est_k = 0;
Temp_out_est_k = 25;
d_est_k = 0;
Temp_heat_sim_k = 0;
Temp_out_sim_k = 28;
d_sim_k = -0.5;

u_guess = zeros(N_pred,1) + u_init;
u_opt_km1 = u_init;

% setpoint轨迹
Temp_sp_array = Temp_sp_const * ones(1,N_sim);

% 存储结果
Temp_out_sp_plot_array = zeros(1,N_sim);
Temp_out_sim_plot_array = zeros(1,N_sim);
u_plot_array = zeros(1,N_sim);

%----------------------------------
% 卡尔曼滤波器观测器设置
A_est = [-1/theta_const, (gain+0.1*randn)/theta_const; 0,0];
C_est = [1 0];
n_est = length(A_est);
Tr_est = 5;
T_est = Tr_est/n_est;
estimpoly = [T_est*T_est,sqrt(2)*T_est,1];
eig_est = roots(estimpoly);
K_est = acker(A_est',C_est',eig_est)';

%----------------------------------
% fmincon约束
A = [];
B = [];
Aeq = [];
Beq = [];
u_max = 220;
u_min = 0;
u_ub = zeros(1,N_pred) + u_max;
u_lb = zeros(1,N_pred) + u_min;

%----------------------------------
% 主仿真循环
for k = 1:(N_sim-N_pred)
    % 估计输入扰动
    e_est_k = Temp_out_sim_k - Temp_out_est_k;
    dTemp_heat_est_dt_k = (1/theta_const)*(-Temp_heat_est_k + (gain+0.1*randn)*(delay_array(end) + d_est_k)) + K_est(1)*e_est_k;
    dd_est_dt_k = 0 + K_est(2)*e_est_k;

    Temp_heat_est_k = Temp_heat_est_k + Ts*dTemp_heat_est_dt_k;
    d_est_k = d_est_k + Ts*dd_est_dt_k;
    Temp_out_est_k = Temp_heat_est_k + Temp_env_k;

    % 给优化器提供参考轨迹
    Temp_sp_to_mpc_array = Temp_sp_array(k:k+N_pred);

    % 构造优化目标函数
    state_est.Temp_heat_est_k = Temp_heat_est_k;
    state_est.d_est_k = d_est_k;

    fun_handle = @(u) fun_objectfunction_mpc_airheater(...
        u, u_opt_km1, state_est, Temp_env_k, Temp_sp_to_mpc_array, model_params, mpc_costs, N_pred, Ts);

    fmincon_options = optimoptions(@fmincon,'display','none');
    [u_opt,~,~,~,~,~,~] = fmincon(fun_handle,u_guess,A,B,Aeq,Beq,u_lb,u_ub,@fun_constraints_mpc_airheater,fmincon_options);

    u_guess = u_opt;
    u_k = u_opt(1);
    u_opt_km1 = u_k;

    % 应用控制信号
    delay_array = [u_k, delay_array(1:end-1)];
    dTemp_heat_sim_dt_k = (1/theta_const)*(-Temp_heat_sim_k + (gain+0.1*randn)*(delay_array(end) + d_sim_k));
    Temp_heat_sim_k = Temp_heat_sim_k + Ts*dTemp_heat_sim_dt_k;
    Temp_out_sim_k = Temp_heat_sim_k + Temp_env_k;

    % 保存数据
    Temp_out_sp_plot_array(k) = Temp_sp_array(k);
    Temp_out_sim_plot_array(k) = Temp_out_sim_k;
    u_plot_array(k) = u_k;
end

%----------------------------------
% 仿真结束，计算cost

%tracking_error = sum((Temp_out_sim_plot_array - Temp_out_sp_plot_array).^2);
%control_input_variation = sum(diff(u_plot_array).^2);

time_weight = 1:N_sim;
tracking_error = sum(time_weight .* abs(Temp_out_sim_plot_array - Temp_out_sp_plot_array));
control_input_variation = sum(diff(u_plot_array).^2);


cost = tracking_error + control_input_variation;

end
