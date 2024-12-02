from bayes_opt import BayesianOptimization
from bayes_opt.util import Colours
import numpy as np
from bayes_opt import BayesianOptimization
from bayes_opt import UtilityFunction
import matlab
import matlab.engine

def bayesopt_objective(aP1,bI1,cD1,dP2):
    #,eI2,fD2
    #reg = RFR(P = int(P),
    # I = int(I)
    # D = int(D)
    # y = int(y))
    #target = float(input("输入指标值："))
    eng = matlab.engine.start_matlab()
    #t = eng.mtest(4, 2)
    target = eng.untitled(aP1,bI1,cD1,dP2)
    # 只能求最大值
    #print(type(target),target)
    return -target
para_grid_simple = {'aP1':(1,10),
                    'bI1':(1,10),
                    'cD1':(1,10),
                    'dP2':(1,10),

                    }
#'eI2':(1,10),
#'fD2':(1,10)
def bayes_opt(init_points,n_iter):
    opt = BayesianOptimization(bayesopt_objective,
                               para_grid_simple,
                               #random_state=1412
                               )

    opt.maximize(init_points = init_points,
                 n_iter = n_iter
                 )
    params_best = opt.max["params"]
    score_best = opt.max["target"]

    print("\n","\n","best params",params_best,
          "\n","\n","best score",score_best)

    return params_best,score_best
if __name__ == "__main__":

    print(Colours.green("--- Optimizing ---"))
    #bayesopt_objective(1,1,1)
    bayes_opt(2,2)

