from bayes_opt import BayesianOptimization
# from bayes_opt.util import Colours
import matlab.engine

def bayesopt_objective(C_e, C_du, t_pred_horizon):
    eng = matlab.engine.start_matlab()
    eng.addpath(r'D:\wty study\MPC-Bayesian')
    target = eng.calculate_error(float(C_e), float(C_du), int(t_pred_horizon))

    return -target

para_grid_simple = {
    'C_e': (1, 10),
    'C_du': (1, 50),
    't_pred_horizon': (1, 10),
}

def bayes_opt(init_points, n_iter):
    opt = BayesianOptimization(
        f=bayesopt_objective,
        pbounds=para_grid_simple,
        verbose=2
    )

    opt.maximize(
        init_points=init_points,
        n_iter=n_iter
    )

    params_best = opt.max["params"]
    score_best = opt.max["target"]

    print("\n", "\n", "best params", params_best,
          "\n", "\n", "best score", score_best)

    return params_best, score_best

if __name__ == "__main__":
    # print(Colours.green("--- Optimizing ---"))
    bayes_opt(2, 8)
