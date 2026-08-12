#ifndef SLS_QSF_CORE_QSF_C_API_H
#define SLS_QSF_CORE_QSF_C_API_H

#ifdef __cplusplus
extern "C" {
#endif

// 基础 QSF 控制器 C ABI。
// z: 12 维状态，k: 10 维增益，param: [load_mass, mav_mass, cable_length, gravity]。
// ref_1/ref_2/ref_3: 每轴 [pos, vel, acc, jerk, snap]，force 输出 3 维 QSF 力。
int sls_qsf_controller(const double *z, const double *k,
                       const double *param, const double *ref_1,
                       const double *ref_2, const double *ref_3, double *force);

// 带积分状态的 QSF 控制器 C ABI，额外输出 xi_dot 供外层积分。
int sls_qsf_integral_controller(const double *z, const double *k,
                                const double *param, const double *ref_1,
                                const double *ref_2, const double *ref_3,
                                double *force, double *xi_dot);

// 几何参考形式的 QSF 控制器 C ABI，主要用于和原始生成代码对照。
int sls_qsf_geometric_controller(const double *z, const double *k,
                                 const double *param, const double *ref,
                                 double t, double *force);

// 返回算法核心版本号，便于日志确认当前加载的共享库。
const char *sls_qsf_core_version(void);

#ifdef __cplusplus
}
#endif

#endif
