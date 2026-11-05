# SPDX-FileCopyrightText: 2025-2026 gurrier <https://github.com/gurrier>
# SPDX-FileCopyrightText: 2022 WeekendWarrior1 <https://github.com/WeekendWarrior1>
# SPDX-License-Identifier: GPL-3.0-only
import esphome.codegen as cg
from esphome.components import ble_client

DEPENDENCIES = ["ble_client"]
AUTO_LOAD = ["sensor"]

powerpal_ble_ns = cg.esphome_ns.namespace("powerpal_ble")
Powerpal = powerpal_ble_ns.class_("Powerpal", ble_client.BLEClientNode, cg.Component)
