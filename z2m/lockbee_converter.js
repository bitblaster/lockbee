'use strict';

/**
 * Zigbee2MQTT external converter for LockBee
 *
 * Exposes only the features actually supported by the firmware:
 *   - lock / unlock
 *   - unlock_with_timeout (ZCL command 0x03, firmware auto-relocks after N seconds)
 *   - lock state reporting (locked / unlocked / not_fully_locked)
 *
 * Installation:
 *   Copy this file to your Zigbee2MQTT external_converters directory, then
 *   add to configuration.yaml:
 *
 *     external_converters:
 *       - lockbee_converter.mjs
 */

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const e = exposes.presets;
const ea = exposes.access;

/*
 * Custom toZigbee converter for UnlockWithTimeout (ZCL command 0x03).
 * Payload: uint16 LE = timeout in seconds.
 * The firmware re-locks automatically after the timeout; no HA automation needed.
 */
const tzUnlockWithTimeout = {
    key: ['unlock_with_timeout'],
    convertSet: async (entity, key, value, meta) => {
        const timeout = parseInt(value, 10);
        await entity.command(
            'closuresDoorLock',
            'unlockWithTimeout',
            {timeout},
            {disableDefaultResponse: true},
        );
        return {state: {unlock_with_timeout: timeout}};
    },
};

/*
 * Custom converter for EnableOneTouchLocking (ZCL attribute 0x0029).
 * Controls whether the physical touch buttons are active.
 * Value is persisted in NVS on the device.
 */
const tzTouchEnabled = {
    key: ['touch_enabled'],
    convertSet: async (entity, key, value, meta) => {
        await entity.write('closuresDoorLock', {enableOneTouchLocking: value ? 1 : 0});
        return {state: {touch_enabled: value}};
    },
    convertGet: async (entity, key, meta) => {
        await entity.read('closuresDoorLock', ['enableOneTouchLocking']);
    },
};

const fzTouchEnabled = {
    cluster: 'closuresDoorLock',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data.hasOwnProperty('enableOneTouchLocking')) {
            return {touch_enabled: msg.data['enableOneTouchLocking'] === 1};
        }
    },
};

const definition = {
    zigbeeModel: ['LockBee-v1'],
    model: 'LockBee-v1',
    vendor: 'LockBee',
    description: 'ESP32-H2 smart lock actuator (NEMA17 + TMC2209 + StallGuard)',
    fromZigbee: [fzTouchEnabled],
    toZigbee: [tz.lock, tzUnlockWithTimeout, tzTouchEnabled],
    exposes: [
        /* No state reporting — no physical position sensor.
         * Home Assistant will use optimistic mode (assumes state = last command). */
        exposes.binary('state', ea.SET, 'LOCK', 'UNLOCK')
            .withDescription('Lock or unlock the door'),
        exposes.numeric('unlock_with_timeout', ea.SET)
            .withValueMin(1)
            .withValueMax(3600)
            .withUnit('s')
            .withDescription('Unlock and automatically re-lock after N seconds (1–3600)'),
        exposes.binary('touch_enabled', ea.ALL, true, false)
            .withDescription('Enable or disable the physical touch buttons'),
    ],
};

module.exports = definition;
