#!/usr/bin/env python3
"""Callbacks que no tiran el nodo. Python puro (sin rclpy) para poder probarlo.

En rclpy una excepción adentro de un callback de suscripción, timer o
servicio sale por `executor.spin()` y termina el proceso. Con el dron en el
aire eso es lo peor que puede pasar: `mission_node` deja de mandar setpoints
(PX4 sale de OFFBOARD a su failsafe) o `mapper_node` deja de publicar
sectores (la misión se queda ciega). Un mensaje malformado de un driver o un
bug en un caso raro no puede valer eso.

`GuardedNodeMixin` reemplaza `create_subscription`, `create_timer` y
`create_service` para que todo callback registrado quede envuelto: la
excepción se registra (con la traza la primera vez, después contada y
limitada a una línea cada 5 s por callback) y el ejecutor sigue. Los servicios
responden `success = False` con el error en vez de colgar al cliente.
"""

import threading
import time
import traceback


class CallbackGuard:
    """Cuenta y registra las excepciones de un callback sin dejarlas pasar."""

    def __init__(self, what, log_error, period=5.0, clock=time.monotonic):
        self.what = what
        self._log_error = log_error
        self.period = float(period)
        self._clock = clock
        self.errors = 0
        self._last_log = -float('inf')
        self._suppressed = 0
        self._lock = threading.Lock()

    def report(self, exc):
        with self._lock:
            self.errors += 1
            now = self._clock()
            first = self.errors == 1
            if not first and now - self._last_log < self.period:
                self._suppressed += 1
                return
            suppressed, self._suppressed = self._suppressed, 0
            self._last_log = now
        if first:
            detail = ''.join(traceback.format_exception(type(exc), exc, exc.__traceback__))
            self._log_error(f'excepción en {self.what} (se descarta y se sigue): {exc!r}\n{detail}')
        else:
            extra = f' (+{suppressed} más sin mostrar)' if suppressed else ''
            self._log_error(f'excepción en {self.what}: {exc!r}; {self.errors} en total{extra}')

    def wrap(self, fn, service=False):
        def guarded(*args):
            try:
                return fn(*args)
            except Exception as exc:  # noqa: BLE001 - es justamente el propósito
                self.report(exc)
                if service and len(args) >= 2:
                    response = args[1]
                    if hasattr(response, 'success'):
                        response.success = False
                    if hasattr(response, 'message'):
                        response.message = f'error interno: {exc!r}'
                    return response
                return None
        guarded.__wrapped__ = fn
        return guarded


class GuardedNodeMixin:
    """Mezclar ANTES de `rclpy.node.Node`: `class MiNodo(GuardedNodeMixin, Node)`."""

    def _guard_log(self, text):
        try:
            self.get_logger().error(text)
        except Exception:  # noqa: BLE001 - el logger no puede ser otra fuente de caídas
            pass

    def _make_guard(self, what):
        guards = self.__dict__.setdefault('_callback_guards', {})
        guard = CallbackGuard(what, self._guard_log)
        guards[f'{what}#{len(guards)}'] = guard
        return guard

    def callback_errors(self):
        """{callback: excepciones atrapadas} (para diagnóstico y pruebas)."""
        return {k: g.errors for k, g in self.__dict__.get('_callback_guards', {}).items()}

    def create_subscription(self, msg_type, topic, callback, *args, **kwargs):
        guard = self._make_guard(f'la suscripción a {topic}')
        return super().create_subscription(msg_type, topic, guard.wrap(callback), *args, **kwargs)

    def create_timer(self, timer_period_sec, callback, *args, **kwargs):
        name = getattr(callback, '__name__', 'timer')
        guard = self._make_guard(f'el timer {name}')
        return super().create_timer(timer_period_sec, guard.wrap(callback), *args, **kwargs)

    def create_service(self, srv_type, srv_name, callback, *args, **kwargs):
        guard = self._make_guard(f'el servicio {srv_name}')
        return super().create_service(srv_type, srv_name, guard.wrap(callback, service=True),
                                      *args, **kwargs)
