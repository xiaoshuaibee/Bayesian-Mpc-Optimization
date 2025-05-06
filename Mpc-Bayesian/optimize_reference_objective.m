function ref_error = optimize_reference_objective(ref_values, gain, theta_const, theta_delay)
    % 模拟误差计算逻辑
    Temp_sp_const = 24;  % 目标温度
    ref_error = sum((ref_values - Temp_sp_const).^2); %
end
