#!/usr/bin/env python3
import subprocess
import tkinter as tk
from tkinter import messagebox


SERVICE = "start_app_node.service"


def systemctl(*args):
    result = subprocess.run(
        ["systemctl", *args, SERVICE],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.returncode, result.stdout.strip(), result.stderr.strip()


def privileged_systemctl(*args):
    return subprocess.run(
        ["pkexec", "systemctl", *args, SERVICE],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


class RobotStackPower(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot Stack Power")
        self.resizable(False, False)
        self.configure(padx=18, pady=18)

        self.status_var = tk.StringVar(value="Checking...")
        self.boot_var = tk.StringVar(value="")

        tk.Label(self, text="Robot Stack", font=("Sans", 18, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w"
        )
        tk.Label(self, textvariable=self.status_var, font=("Sans", 12)).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(10, 2)
        )
        tk.Label(self, textvariable=self.boot_var, font=("Sans", 10)).grid(
            row=2, column=0, columnspan=2, sticky="w", pady=(0, 14)
        )

        self.start_button = tk.Button(
            self, text="Start Stack", width=16, command=self.start_stack
        )
        self.start_button.grid(row=3, column=0, padx=(0, 8), pady=4)

        self.stop_button = tk.Button(
            self, text="Stop Stack", width=16, command=self.stop_stack
        )
        self.stop_button.grid(row=3, column=1, padx=(8, 0), pady=4)

        self.enable_button = tk.Button(
            self, text="Start on Boot", width=16, command=self.enable_boot
        )
        self.enable_button.grid(row=4, column=0, padx=(0, 8), pady=(10, 4))

        self.disable_button = tk.Button(
            self, text="Do Not Start on Boot", width=16, command=self.disable_boot
        )
        self.disable_button.grid(row=4, column=1, padx=(8, 0), pady=(10, 4))

        self.refresh_button = tk.Button(
            self, text="Refresh", width=16, command=self.refresh
        )
        self.refresh_button.grid(row=5, column=0, columnspan=2, pady=(12, 0))

        self.refresh()
        self.after(5000, self.refresh_periodically)

    def refresh_periodically(self):
        self.refresh(show_errors=False)
        self.after(5000, self.refresh_periodically)

    def refresh(self, show_errors=True):
        active_code, active, active_err = systemctl("is-active")
        enabled_code, enabled, enabled_err = systemctl("is-enabled")

        if active_code == 0:
            self.status_var.set("Status: running")
            self.start_button.configure(state=tk.DISABLED)
            self.stop_button.configure(state=tk.NORMAL)
        elif active in {"inactive", "failed", "deactivating", "activating"}:
            self.status_var.set(f"Status: {active}")
            self.start_button.configure(state=tk.NORMAL)
            self.stop_button.configure(state=tk.DISABLED if active == "inactive" else tk.NORMAL)
        else:
            self.status_var.set("Status: unknown")
            self.start_button.configure(state=tk.NORMAL)
            self.stop_button.configure(state=tk.NORMAL)
            if show_errors and active_err:
                messagebox.showerror("Status Error", active_err)

        if enabled_code == 0:
            self.boot_var.set("Boot: starts automatically")
            self.enable_button.configure(state=tk.DISABLED)
            self.disable_button.configure(state=tk.NORMAL)
        elif enabled in {"disabled", "static", "masked"}:
            self.boot_var.set(f"Boot: {enabled}")
            self.enable_button.configure(state=tk.NORMAL)
            self.disable_button.configure(state=tk.DISABLED if enabled == "disabled" else tk.NORMAL)
        else:
            self.boot_var.set("Boot: unknown")
            self.enable_button.configure(state=tk.NORMAL)
            self.disable_button.configure(state=tk.NORMAL)
            if show_errors and enabled_err:
                messagebox.showerror("Boot Status Error", enabled_err)

    def run_action(self, label, *args):
        self.status_var.set(f"{label}...")
        self.update_idletasks()
        result = privileged_systemctl(*args)
        self.refresh(show_errors=False)
        if result.returncode != 0:
            details = result.stderr.strip() or result.stdout.strip() or "Command was cancelled or failed."
            messagebox.showerror(label, details)

    def start_stack(self):
        self.run_action("Starting robot stack", "start")

    def stop_stack(self):
        if messagebox.askyesno(
            "Stop Robot Stack",
            "Stop the robot stack now to reduce idle power use?",
        ):
            self.run_action("Stopping robot stack", "stop")

    def enable_boot(self):
        self.run_action("Enabling boot start", "enable")

    def disable_boot(self):
        self.run_action("Disabling boot start", "disable")


if __name__ == "__main__":
    RobotStackPower().mainloop()
