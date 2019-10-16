# SSH Tunnels

## Commands

> Change _CIP_ for yours.
> Change server.addr.ca to the server you want to connect using ssh.

### With Shell
```bash
ssh -L 1183:localhost:1183 CIP@server.addr.ca
```

### Without Shell
```bash
ssh -nNT -L 1183:localhost:1183 CIP@server.addr.ca
```
