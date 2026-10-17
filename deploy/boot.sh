#!/bin/zsh
# Explicitly switch the boot default. Never enables both robot stacks.
set -eu

UNIT_SOURCE=$HOME/hwr/deploy/hwr.service
UNIT_TARGET=/etc/systemd/system/hwr.service

case ${1:-status} in
  install)
    sudo install -m 0644 $UNIT_SOURCE $UNIT_TARGET
    sudo install -m 0644 $HOME/hwr/deploy/hwr-ears.service /etc/systemd/system/hwr-ears.service
    sudo systemctl daemon-reload
    print "installed disabled hwr.service (+ hwr-ears.service, started with it)"
    ;;
  enable)
    [[ -f $UNIT_TARGET ]] || { print "run: $0 install"; exit 1; }
    # hwr.service's takeover preflight stops the factory stack before opening
    # the hardware ports.
    sudo systemctl disable start_app_node.service
    sudo systemctl enable --now hwr.service
    print "hwr.service is now the boot default"
    ;;
  disable)
    sudo systemctl disable --now hwr.service 2>/dev/null || true
    sudo systemctl enable --now start_app_node.service
    print "factory stack restored as the boot default"
    ;;
  status)
    systemctl is-enabled hwr.service 2>/dev/null || true
    systemctl --no-pager --full status hwr.service hwr-ears.service start_app_node.service || true
    ;;
  *)
    print "usage: $0 install|enable|disable|status"
    exit 2
    ;;
esac
