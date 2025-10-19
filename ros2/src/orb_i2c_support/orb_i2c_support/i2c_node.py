"""ROS 2 node exposing the ORB I2C interface as a service."""

from __future__ import annotations

import json
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from orb_i2c_interfaces.srv import I2CTransaction

from .backend_factory import create_backend
from .i2c_backend import I2CBackend, I2CBackendError


class I2CManagerNode(Node):
    """Service-based wrapper around the configurable I2C backend."""

    def __init__(self) -> None:
        super().__init__("orb_i2c_manager")

        self.declare_parameter("bus_id", "i2c-1")
        self.declare_parameter("backend", "simulation")
        self.declare_parameter("backend_plugin", "")
        self.declare_parameter("backend_configuration", "{}")
        self.declare_parameter("status_interval_sec", 1.0)

        backend_name = self.get_parameter("backend").get_parameter_value().string_value
        backend_plugin = (
            self.get_parameter("backend_plugin").get_parameter_value().string_value
        )
        backend_config_raw = (
            self.get_parameter("backend_configuration")
            .get_parameter_value()
            .string_value
        )
        bus_id = self.get_parameter("bus_id").get_parameter_value().string_value

        self._backend: I2CBackend = create_backend(backend_name, backend_plugin or None)
        self._configure_backend(bus_id, backend_config_raw)

        self._transaction_srv = self.create_service(
            I2CTransaction,
            "i2c/transaction",
            self._handle_transaction,
        )

        self._status_pub = self.create_publisher(String, "orb_i2c/status", 10)
        interval = (
            self.get_parameter("status_interval_sec")
            .get_parameter_value()
            .double_value
        )
        self._status_timer = (
            self.create_timer(interval, self._publish_status) if interval > 0 else None
        )

        self.get_logger().info(
            "I2C manager ready on bus '%s' using backend '%s'%s",
            bus_id,
            backend_name,
            f" ({backend_plugin})" if backend_plugin else "",
        )

    def _configure_backend(self, bus_id: str, backend_config_raw: str) -> None:
        configuration = self._parse_configuration(backend_config_raw)
        try:
            self._backend.configure(bus_id=bus_id, configuration=configuration)
        except I2CBackendError as exc:
            self.get_logger().error("Backend configuration failed: %s", exc)
            raise

    def _parse_configuration(self, raw_value: str) -> Dict[str, Any]:
        if not raw_value or raw_value.strip() == "":
            return {}
        try:
            parsed = json.loads(raw_value)
            if isinstance(parsed, dict):
                return parsed
        except json.JSONDecodeError as exc:
            self.get_logger().warning(
                "Unable to parse backend configuration JSON ('%s'): %s. "
                "Falling back to empty configuration.",
                raw_value,
                exc,
            )
        return {}

    def _handle_transaction(
        self, request: I2CTransaction.Request, response: I2CTransaction.Response
    ) -> I2CTransaction.Response:
        try:
            success, read_data, error_message = self._backend.perform_transaction(
                address=int(request.address),
                write_data=request.write_data,
                read_length=int(request.read_length),
                timeout=float(request.timeout),
            )
            response.success = success
            response.read_data = list(read_data)
            response.error_message = error_message
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().exception("I2C transaction failed")
            response.success = False
            response.read_data = []
            response.error_message = str(exc)
        return response

    def _publish_status(self) -> None:
        backend_name = type(self._backend).__name__
        bus_id = self._backend.bus_id or "<unconfigured>"
        msg = String()
        msg.data = f"backend={backend_name} bus={bus_id}"
        self._status_pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = I2CManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
