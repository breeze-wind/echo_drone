"""`drown` QSF/SLS 生成算法的 Python 调用封装。"""

import ctypes
import os

from ament_index_python.packages import get_package_prefix


class QsfCoreError(RuntimeError):
    """QSF 动态库加载或调用失败时抛出的异常。"""
    pass


def _array(values, size):
    """把 Python list 转成 C ABI 需要的定长 double 数组。"""
    if len(values) != size:
        raise QsfCoreError('expected %d values, got %d' % (size, len(values)))
    return (ctypes.c_double * size)(*[float(v) for v in values])


class QsfCore:
    """通过 ctypes 调用 `sls_qsf_core` 共享库。"""

    def __init__(self):
        # Python 版控制器通过 ament 安装前缀查找共享库，避免写死 build/install 路径。
        prefix = get_package_prefix('sls_qsf_core')
        lib_path = os.path.join(prefix, 'lib', 'libsls_qsf_core.so')
        if not os.path.exists(lib_path):
            raise QsfCoreError('QSF core library not found: %s' % lib_path)

        self.lib = ctypes.CDLL(lib_path)
        double_p = ctypes.POINTER(ctypes.c_double)

        # 这里声明 C ABI 的参数类型，尺寸检查留给每个 wrapper 方法里的 _array。
        self.lib.sls_qsf_controller.argtypes = [
            double_p, double_p, double_p, double_p, double_p, double_p, double_p]
        self.lib.sls_qsf_controller.restype = ctypes.c_int

        self.lib.sls_qsf_integral_controller.argtypes = [
            double_p, double_p, double_p, double_p, double_p, double_p,
            double_p, double_p]
        self.lib.sls_qsf_integral_controller.restype = ctypes.c_int

        self.lib.sls_qsf_geometric_controller.argtypes = [
            double_p, double_p, double_p, double_p, ctypes.c_double, double_p]
        self.lib.sls_qsf_geometric_controller.restype = ctypes.c_int

        self.lib.sls_qsf_core_version.argtypes = []
        self.lib.sls_qsf_core_version.restype = ctypes.c_char_p

    def version(self):
        return self.lib.sls_qsf_core_version().decode('ascii')

    def qsf(self, state, gains, params, ref_x, ref_y, ref_z):
        """调用基础 QSF 控制器，返回 NED 语义下的三轴力。"""
        force = (ctypes.c_double * 3)()
        ret = self.lib.sls_qsf_controller(
            _array(state, 12), _array(gains, 10), _array(params, 4),
            _array(ref_x, 5), _array(ref_y, 5), _array(ref_z, 5), force)
        if ret != 0:
            raise QsfCoreError('sls_qsf_controller failed with code %d' % ret)
        return [force[0], force[1], force[2]]

    def qsf_integral(self, state, gains, params, ref_x, ref_y, ref_z):
        """调用带积分状态的 QSF 控制器，额外返回积分状态导数。"""
        force = (ctypes.c_double * 3)()
        xi_dot = (ctypes.c_double * 3)()
        ret = self.lib.sls_qsf_integral_controller(
            _array(state, 15), _array(gains, 13), _array(params, 4),
            _array(ref_x, 5), _array(ref_y, 5), _array(ref_z, 5),
            force, xi_dot)
        if ret != 0:
            raise QsfCoreError(
                'sls_qsf_integral_controller failed with code %d' % ret)
        return [force[0], force[1], force[2]], [xi_dot[0], xi_dot[1], xi_dot[2]]

    def qsf_geometric(self, state, gains, params, ref, elapsed):
        """调用几何参考形式的 QSF 控制器。"""
        force = (ctypes.c_double * 3)()
        ret = self.lib.sls_qsf_geometric_controller(
            _array(state, 12), _array(gains, 10), _array(params, 4),
            _array(ref, 12), float(elapsed), force)
        if ret != 0:
            raise QsfCoreError(
                'sls_qsf_geometric_controller failed with code %d' % ret)
        return [force[0], force[1], force[2]]
