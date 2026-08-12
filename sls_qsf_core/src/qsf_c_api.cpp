#include "sls_qsf_core/qsf_c_api.h"

#include "QSFController.h"
#include "QSFGeometricController.h"
#include "QSFIntegralController.h"

// 这个文件是手写的薄包装层，负责把生成代码封装成稳定 C ABI。
// 上层 C++/Python 只依赖 qsf_c_api.h，避免直接耦合 generated 目录里的函数签名。
extern "C" {

int sls_qsf_controller(const double *z, const double *k,
                       const double *param, const double *ref_1,
                       const double *ref_2, const double *ref_3, double *force)
{
  // 所有入口统一做空指针检查，调用方可通过非零返回值退回 PD。
  if (!z || !k || !param || !ref_1 || !ref_2 || !ref_3 || !force) {
    return -1;
  }
  QSFController(z, k, param, ref_1, ref_2, ref_3, force);
  return 0;
}

int sls_qsf_integral_controller(const double *z, const double *k,
                                const double *param, const double *ref_1,
                                const double *ref_2, const double *ref_3,
                                double *force, double *xi_dot)
{
  if (!z || !k || !param || !ref_1 || !ref_2 || !ref_3 || !force || !xi_dot) {
    return -1;
  }
  QSFIntegralController(z, k, param, ref_1, ref_2, ref_3, force, xi_dot);
  return 0;
}

int sls_qsf_geometric_controller(const double *z, const double *k,
                                 const double *param, const double *ref,
                                 double t, double *force)
{
  if (!z || !k || !param || !ref || !force) {
    return -1;
  }
  QSFGeometricController(z, k, param, ref, t, force);
  return 0;
}

const char *sls_qsf_core_version(void)
{
  // 版本字符串进入控制器日志和 /sls_circle/status，用于区分不同算法导出版本。
  return "drown-qsf-2025-02-02-scalar";
}

}  // extern "C"
