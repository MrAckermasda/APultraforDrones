"""Joint jog validation and asynchronous parameter editing; no motion commands."""
import math
import time

JOINT_SETTINGS = {'joint_step': (int, 1, 20), 'joint_duration': (float, 0.02, 30.0)}


def validate_joint_setting(name, value):
    expected, lower, upper = JOINT_SETTINGS[name]
    if type(value) is not expected or not lower <= value <= upper or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite {expected.__name__} in [{lower}, {upper}]')
    return value


def parse_joint_settings(step, duration):
    try:
        values = {'joint_step': int(step), 'joint_duration': float(duration)}
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError('步长需为整数，时间需为有限小数') from exc
    for name, value in values.items():
        validate_joint_setting(name, value)
    return values


class JogParameterClient:
    """One pending operation; polling and notification run on the GUI thread."""
    def __init__(self, node, target, notify, timeout=3.0):
        from rcl_interfaces.srv import GetParameters, SetParametersAtomically
        self.target = '/' + target.strip('/')
        self.notify = notify
        self.timeout = timeout
        self.get_client = node.create_client(GetParameters, self.target + '/get_parameters')
        self.set_client = node.create_client(SetParametersAtomically, self.target + '/set_parameters_atomically')
        self.pending = None

    def request(self, action, values=None):
        from rcl_interfaces.srv import GetParameters, SetParametersAtomically
        from rclpy.parameter import Parameter
        if self.pending is not None:
            return False
        if action not in ('read', 'apply'):
            raise ValueError('Unknown parameter operation')
        values = dict(values or {})
        if action == 'apply':
            if set(values) != set(JOINT_SETTINGS):
                raise ValueError('Both joint settings are required')
            for name, value in values.items():
                validate_joint_setting(name, value)
            client = self.set_client
            request = SetParametersAtomically.Request(parameters=[
                Parameter(name, value=value).to_parameter_msg() for name, value in values.items()])
        else:
            client = self.get_client
            request = GetParameters.Request(names=list(JOINT_SETTINGS))
        if not client.service_is_ready():
            self.notify(action, False, values, f'{self.target} 参数服务未就绪；请确认新版手柄节点已启动')
            return False
        try:
            future = client.call_async(request)
        except Exception as exc:
            self.notify(action, False, values, f'参数请求失败：{exc}')
            return False
        self.pending = (future, action, values, time.monotonic() + self.timeout)
        return True

    def poll(self):
        from rcl_interfaces.msg import ParameterType
        if self.pending is None:
            return
        future, action, values, deadline = self.pending
        if not future.done():
            if time.monotonic() < deadline:
                return
            self.pending = None
            future.cancel()
            message = '请求超时；未确认当前值，请点击“读取当前值”'
            if action == 'apply':
                message += '。远端可能已应用，勿据此认定失败或自动重试'
            self.notify(action, False, values, message)
            return
        self.pending = None
        try:
            response = future.result()
            if action == 'read':
                if len(response.values) != 2 or [v.type for v in response.values] != [
                        ParameterType.PARAMETER_INTEGER, ParameterType.PARAMETER_DOUBLE]:
                    raise ValueError('远端缺少参数或类型不匹配')
                values = {'joint_step': response.values[0].integer_value,
                          'joint_duration': response.values[1].double_value}
                for name, value in values.items():
                    validate_joint_setting(name, value)
                self.notify(action, True, values, '已读取节点当前值')
            elif response.result.successful:
                self.notify(action, True, values, '节点确认已应用；后续关节点动使用新值')
            else:
                self.notify(action, False, values,
                            '节点拒绝：' + response.result.reason + '（旧版本只读参数需先重启新版节点）')
        except Exception as exc:
            self.notify(action, False, values, f'参数响应异常，当前值未确认：{exc}')

    def close(self):
        if self.pending is not None:
            self.pending[0].cancel()
            self.pending = None
