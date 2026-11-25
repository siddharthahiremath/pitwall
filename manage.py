#!/usr/bin/env python3
"""Account management for PITWALL.

  python3 manage.py init                      create the default accounts (edit PLAYERS below first)
  python3 manage.py add NAME [--admin]        add one account
  python3 manage.py passwd NAME               reset a password
  python3 manage.py list

Passwords are generated randomly, stored only as salted PBKDF2 hashes in data/accounts.json,
and printed once into data/ACCOUNTS.local.txt for you to hand out. data/ is never committed.
Restart the server after changing accounts.
"""
import json
import os
import secrets
import sys

from server import ACCOUNTS_FILE, DATA_DIR, hash_pw, load_json, save_json

# name, admin?, car colour
PLAYERS = [
    ("sid", True, "#ff2d55"),
    ("logan", False, "#00c2ff"),
    ("will", False, "#ffb800"),
    ("nathan", False, "#2bd96b"),
]
OUT = os.path.join(DATA_DIR, "ACCOUNTS.local.txt")


def set_pw(accounts, name):
    pw = secrets.token_urlsafe(10)
    salt = secrets.token_hex(16)
    accounts[name].update({"salt": salt, "hash": hash_pw(pw, salt)})
    return pw


def write_out(lines):
    with open(OUT, "a") as f:
        f.write("\n".join(lines) + "\n")
    os.chmod(OUT, 0o600)


def main():
    os.makedirs(os.path.join(DATA_DIR, "bots"), exist_ok=True)
    acc = load_json(ACCOUNTS_FILE, {})
    cmd = sys.argv[1] if len(sys.argv) > 1 else "list"
    lines = []
    if cmd == "init":
        for name, admin, color in PLAYERS:
            if name in acc:
                print("exists:", name)
                continue
            acc[name] = {"admin": admin, "color": color}
            lines.append("%-8s %s%s" % (name, set_pw(acc, name), "   (admin)" if admin else ""))
    elif cmd == "add":
        name = sys.argv[2].lower()
        acc[name] = {"admin": "--admin" in sys.argv, "color": None}
        lines.append("%-8s %s" % (name, set_pw(acc, name)))
    elif cmd == "passwd":
        name = sys.argv[2].lower()
        if name not in acc:
            sys.exit("no such account")
        lines.append("%-8s %s   (reset)" % (name, set_pw(acc, name)))
    for name, a in acc.items():
        if cmd == "list":
            print(name, "(admin)" if a.get("admin") else "")
    save_json(ACCOUNTS_FILE, acc)
    if lines:
        write_out(lines)
        print("Wrote %d password(s) to %s" % (len(lines), OUT))


if __name__ == "__main__":
    main()
