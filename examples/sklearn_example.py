from sklearn.datasets import make_classification
from sklearn.model_selection import cross_val_score
from sklearn.ensemble import RandomForestClassifier as RFC
from sklearn.svm import SVC

from bayes_opt import BayesianOptimization
from bayes_opt.util import Colours


def get_data():
    """Synthetic binary classification dataset.合成二进制分类数据集。"""
    data, targets = make_classification(
        n_samples=1000,
        n_features=45,
        n_informative=12,
        n_redundant=7,
        random_state=134985745,
    )
    return data, targets


def svc_cv(C, gamma, data, targets):
    """SVC cross validation.

    This function will instantiate a SVC classifier with parameters C and
    gamma. Combined with data and targets this will in turn be used to perform
    cross validation. The result of cross validation is returned.
    这个函数将实例化一个带有参数C和gamma的SVC分类器。结合数据和目标，这将依次用于执行交叉验证。返回交叉验证的结果。

    Our goal is to find combinations of C and gamma that maximizes the roc_auc
    metric.
    我们的目标是找到C和伽马的组合，使roc_auc度量最大化。
    """
    estimator = SVC(C=C, gamma=gamma, random_state=2)
    cval = cross_val_score(estimator, data, targets, scoring='roc_auc', cv=4)
    return cval.mean()


def rfc_cv(n_estimators, min_samples_split, max_features, data, targets):
    """Random Forest cross validation.

    This function will instantiate a random forest classifier with parameters
    n_estimators, min_samples_split, and max_features. Combined with data and
    targets this will in turn be used to perform cross validation. The result
    of cross validation is returned.
    这个函数将实例化一个带有参数n_estimators、min_samples_split和max_features的随机森林分类器。结合数据和目标，这将依次用于执行交叉验证。返回交叉验证的结果。

    Our goal is to find combinations of n_estimators, min_samples_split, and
    max_features that minimzes the log loss.
    我们的目标是找到n_estimators、min_samples_split和max_features的组合，使日志损失最小化。
    """
    estimator = RFC(
        n_estimators=n_estimators,
        min_samples_split=min_samples_split,
        max_features=max_features,
        random_state=2
    )
    cval = cross_val_score(estimator, data, targets,
                           scoring='neg_log_loss', cv=4)
    return cval.mean()


def optimize_svc(data, targets):
    """Apply Bayesian Optimization to SVC parameters.
    将贝叶斯优化应用于SVC参数。"""
    def svc_crossval(expC, expGamma):
        """Wrapper of SVC cross validation.
        SVC交叉验证的包装

        Notice how we transform between regular and log scale. While this
        is not technically necessary, it greatly improves the performance
        of the optimizer.
        注意我们是如何在正则尺度和对数尺度之间转换的。虽然这在技术上不是必需的，但它极大地提高了优化器的性能。
        """
        C = 10 ** expC
        gamma = 10 ** expGamma
        return svc_cv(C=C, gamma=gamma, data=data, targets=targets)

    optimizer = BayesianOptimization(
        f=svc_crossval,
        pbounds={"expC": (-3, 2), "expGamma": (-4, -1)},
        random_state=1234,
        verbose=2
    )
    optimizer.maximize(n_iter=10)
    """n_iter：要执行多少步贝叶斯优化。步数越多，就越有可能找到一个好的最大值。
    init_points：要执行多少步随机探索。随机探索可以通过使探索空间多样化来提供帮助。
    """

    print("Final result:", optimizer.max)


def optimize_rfc(data, targets):
    """Apply Bayesian Optimization to Random Forest parameters.
    应用贝叶斯优化随机森林参数。"""
    def rfc_crossval(n_estimators, min_samples_split, max_features):
        """Wrapper of RandomForest cross validation.

        Notice how we ensure n_estimators and min_samples_split are casted
        to integer before we pass them along. Moreover, to avoid max_features
        taking values outside the (0, 1) range, we also ensure it is capped
        accordingly.
        注意，在传递n_estimators和min_samples_split之前，我们是如何确保它们被强制转换为整数的。此外，为了避免max_features的值超出(0,1)范围，我们还确保对其设置相应的上限。
        """
        return rfc_cv(
            n_estimators=int(n_estimators),
            min_samples_split=int(min_samples_split),
            max_features=max(min(max_features, 0.999), 1e-3),
            data=data,
            targets=targets,
        )

    optimizer = BayesianOptimization(
        f=rfc_crossval,
        pbounds={
            "n_estimators": (10, 250),
            "min_samples_split": (2, 25),
            "max_features": (0.1, 0.999),
        },
        random_state=1234,
        verbose=2
    )
    optimizer.maximize(n_iter=15)

    print("Final result:", optimizer.max)

if __name__ == "__main__":
    data, targets = get_data()

    print(Colours.yellow("--- Optimizing SVM ---"))
    optimize_svc(data, targets)

    print(Colours.green("--- Optimizing Random Forest ---"))
    optimize_rfc(data, targets)
