# Thread Device Communication Testing Guide

## Prerequisites
- Two ESP32-C6 devices flashed with this code
- One configured as Router (default)
- One configured as End Device (optional)
- Serial terminal to each device (use `idf.py monitor`)

## Step 1: Form a Network (Router Device)

On the **first device** (Router), enter these CLI commands:

```bash
# Set network credentials
dataset init new
dataset commit active
ifconfig up
thread start
```

Wait a few seconds, then check status:
```bash
state
# Should show: leader or router
```

Get the network credentials to join from other devices:
```bash
dataset active -x
# Copy this hex string - you'll need it for the second device
```

## Step 2: Join the Network (Second Device)

On the **second device** (End Device or another Router):

```bash
# Set the network credentials (replace <hex_string> with the output from Step 1)
dataset set active <hex_string>
ifconfig up
thread start
```

Wait a few seconds, then check:
```bash
state
# Should show: child, router, or leader
```

## Step 3: Test Communication

### Get IPv6 Addresses

On **both devices**, get their mesh-local addresses:
```bash
ipaddr
# Look for addresses starting with fd (mesh-local)
# Example: fdde:ad00:beef:0:1234:5678:9abc:def0
```

### Send Ping Between Devices

From **Device 1**, ping **Device 2**:
```bash
ping <device2_ipv6_address>
# Example: ping fdde:ad00:beef:0:1234:5678:9abc:def0
```

You should see responses like:
```
16 bytes from fdde:ad00:beef:0:1234:5678:9abc:def0: icmp_seq=1 hlim=64 time=15ms
```

### Check Neighbor List

On **either device**, see connected neighbors:
```bash
neighbor table
# Shows all directly connected Thread devices
```

### Check Router Table (Router only)

On a **Router device**:
```bash
router table
# Shows all routers in the network
```

## Step 4: Verify Device Roles

Check what role each device has:
```bash
state
# Possible values:
# - leader: First router in network
# - router: Additional routers
# - child: End device attached to a router
```

For more details:
```bash
child table    # On router: shows attached children
parent         # On end device: shows the parent router
```

## Troubleshooting

### Devices not joining:
```bash
# Reset and try again
factoryreset
# Then repeat Step 1 or 2
```

### Check radio is working:
```bash
channel
# Should show a channel (11-26)

panid
# Should show a PAN ID
```

### Check network key:
```bash
networkkey
# Both devices should have the same key
```

## Monitoring Communication

Watch real-time logs in the monitor terminal:
- Look for "State changed" messages
- Look for "Attached successfully" or "Detached"
- Ping responses confirm end-to-end communication

## Success Indicators

✅ Both devices show `state` as connected (not disabled/detached)  
✅ `ping` commands get responses  
✅ `neighbor table` shows other device(s)  
✅ End device shows a parent with `parent` command  
✅ Router shows children with `child table` command
