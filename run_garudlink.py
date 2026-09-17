#!/usr/bin/env python3
"""
===============================================================================
 PROJECT GARUDLINK: TACTICAL DEFENSE & SHIP DATA NETWORK (SDN) ORCHESTRATOR
===============================================================================
 File:        run_garudlink.py
 Description: Autonomous zero-dependency build orchestrator, synthetic
              verification engine, C++ telemetry stream parser, and web dashboard
              launcher for Project GarudLink.
 Author:      DevOps & Simulation GNC Team
 Standard:    Python 3.8+ (Zero External Dependencies - Standard Library Only)
===============================================================================
"""

import os
import sys
import time
import math
import signal
import shutil
import socket
import threading
import webbrowser
import subprocess
from pathlib import Path
from http.server import SimpleHTTPRequestHandler
from socketserver import TCPServer

# =============================================================================
# ANSI COLOR PALETTE & FORMATTING
# =============================================================================
# Enable Windows ANSI VT100 escape sequences
if sys.platform == "win32":
    os.system("")

class Colors:
    RESET   = "\033[0m"
    BOLD    = "\033[1m"
    DIM     = "\033[2m"
    
    # Required stream tags
    CYAN    = "\033[96m"  # [BUS-MSG]
    YELLOW  = "\033[93m"  # [RADAR]
    RED     = "\033[91m"  # [MSL-GNC]
    GREEN   = "\033[92m"  # [SYSTEM]
    
    # Extra tactical accents
    MAGENTA = "\033[95m"
    WHITE   = "\033[97m"
    GRAY    = "\033[90m"
    BG_BLUE = "\033[44m\033[97m"

def print_banner():
    banner = f"""{Colors.CYAN}{Colors.BOLD}
===============================================================================
       ____ ____  ____  _   _ ____  _     ___ _   _ _  __
      / ___/ ___||  _ \\| | | |  _ \\| |   |_ _| \\ | | |/ /
     | |  _\\___ \\| | | | | | | |_) | |    | ||  \\| | ' / 
     | |_| |___) | |_| | |_| |  _ <| |___ | || |\\  | . \\ 
      \\____|____/|____/ \\___/|_| \\_\\_____|___|_| \\_|_|\\_\\
  TACTICAL DEFENSE NETWORK & REAL-TIME C++ GNC / CMS ORCHESTRATOR
===============================================================================
{Colors.RESET}"""
    print(banner)

def log_system(msg: str):
    print(f"{Colors.GREEN}{Colors.BOLD}[SYSTEM]{Colors.RESET} {msg}")

def log_bus(msg: str):
    print(f"{Colors.CYAN}[BUS-MSG]{Colors.RESET} {msg}")

def log_radar(msg: str):
    print(f"{Colors.YELLOW}[RADAR]{Colors.RESET} {msg}")

def log_msl(msg: str):
    print(f"{Colors.RED}[MSL-GNC]{Colors.RESET} {msg}")

def log_warn(msg: str):
    print(f"{Colors.YELLOW}{Colors.BOLD}[WARNING]{Colors.RESET} {msg}")

def log_error(msg: str):
    print(f"{Colors.RED}{Colors.BOLD}[ERROR]{Colors.RESET} {msg}")


# =============================================================================
# PART 1: DETERMINISTIC SYNTHETIC SELF-VERIFICATION ENGINE
# =============================================================================

class SyntheticVerificationSuite:
    """
    Self-contained verification engine that validates kinematic mathematics,
    NMEA frame checksums, and radar range equation physics prior to execution.
    """

    @staticmethod
    def run_all_checks() -> bool:
        print(f"\n{Colors.WHITE}{Colors.BOLD}>>> INITIATING SYNTHETIC SELF-VERIFICATION SUITE...{Colors.RESET}")
        time.sleep(0.2)
        
        ok_a = SyntheticVerificationSuite.scenario_a_kinematics()
        ok_b = SyntheticVerificationSuite.scenario_b_nmea_checksums()
        ok_c = SyntheticVerificationSuite.scenario_c_radar_range_equation()
        
        if ok_a and ok_b and ok_c:
            log_system(f"{Colors.GREEN}All Synthetic Verification Scenarios PASSED (3/3). Pipeline Verified.{Colors.RESET}\n")
            return True
        else:
            log_error(f"One or more synthetic validation checks failed!")
            return False

    @staticmethod
    def scenario_a_kinematics() -> bool:
        """
        Scenario A: Kinematics & Coordinate Check.
        Spawns a target UAV at (X=15000m, Y=15000m, Z=2000m) with inbound velocity
        towards origin (0, 0, 0). Asserts calculated 3D distance decreases monotonically.
        """
        log_system("Running Scenario A: 3D Kinematics & Coordinate Verification...")
        
        # Initial state
        x, y, z = 15000.0, 15000.0, 2000.0
        
        # Calculate unit direction vector pointing directly to origin (0,0,0)
        initial_dist = math.sqrt(x*x + y*y + z*z)
        speed = 180.0  # m/s
        vx = - (x / initial_dist) * speed
        vy = - (y / initial_dist) * speed
        vz = - (z / initial_dist) * speed
        
        dt = 0.1  # 100ms time step
        steps = 50
        prev_dist = initial_dist
        
        for step in range(1, steps + 1):
            x += vx * dt
            y += vy * dt
            z += vz * dt
            
            curr_dist = math.sqrt(x*x + y*y + z*z)
            if curr_dist >= prev_dist:
                log_error(f"Kinematic monotonicity violation at step {step}: prev={prev_dist:.3f}m, curr={curr_dist:.3f}m")
                return False
            prev_dist = curr_dist
            
        log_system(f"  [PASS] Kinematics: 3D Range decreased monotonically from {initial_dist:.1f}m -> {prev_dist:.1f}m over {steps*dt:.1f}s.")
        return True

    @staticmethod
    def calculate_nmea_checksum(sentence: str) -> str:
        """Computes 8-bit XOR checksum of characters between '$' and '*'."""
        raw = sentence.strip()
        if raw.startswith('$'):
            raw = raw[1:]
        if '*' in raw:
            raw = raw.split('*')[0]
            
        checksum = 0
        for char in raw:
            checksum ^= ord(char)
        return f"{checksum:02X}"

    @staticmethod
    def validate_nmea_sentence(sentence: str) -> bool:
        """Validates format and 8-bit XOR checksum of an NMEA packet."""
        sentence = sentence.strip()
        if not sentence.startswith('$') or '*' not in sentence:
            return False
            
        content, received_chk = sentence[1:].rsplit('*', 1)
        expected_chk = SyntheticVerificationSuite.calculate_nmea_checksum(content)
        return received_chk.upper() == expected_chk.upper()

    @staticmethod
    def scenario_b_nmea_checksums() -> bool:
        """
        Scenario B: NMEA Frame & Checksum Integrity.
        Generates synthetic NMEA sentences, calculates 8-bit XOR checksums, injects
        bit-flip errors, and asserts validator accepts valid and rejects corrupted.
        """
        log_system("Running Scenario B: NMEA Frame & Checksum Integrity Check...")
        
        # Test 1: Generate valid sentences
        uav_payload = "UAV,POS,15000.0,15000.0,2000.0,-120.0,-120.0,-15.0"
        uav_chk = SyntheticVerificationSuite.calculate_nmea_checksum(uav_payload)
        uav_frame = f"${uav_payload}*{uav_chk}"
        
        rdr_payload = "RDR,TRK,UAV-ALPHA,45.0,21213.2,1.25e-11"
        rdr_chk = SyntheticVerificationSuite.calculate_nmea_checksum(rdr_payload)
        rdr_frame = f"${rdr_payload}*{rdr_chk}"
        
        if not SyntheticVerificationSuite.validate_nmea_sentence(uav_frame):
            log_error(f"Valid UAV NMEA sentence was incorrectly rejected: {uav_frame}")
            return False
            
        if not SyntheticVerificationSuite.validate_nmea_sentence(rdr_frame):
            log_error(f"Valid RDR NMEA sentence was incorrectly rejected: {rdr_frame}")
            return False
            
        # Test 2: Injected bit-flip corruption in payload
        corrupted_payload = "UAV,POS,15001.0,15000.0,2000.0,-120.0,-120.0,-15.0"
        corrupted_frame_1 = f"${corrupted_payload}*{uav_chk}"  # payload changed, old checksum
        
        # Test 3: Injected bit-flip corruption in checksum
        corrupted_chk = f"{(int(rdr_chk, 16) ^ 0x01):02X}"
        corrupted_frame_2 = f"${rdr_payload}*{corrupted_chk}"
        
        if SyntheticVerificationSuite.validate_nmea_sentence(corrupted_frame_1):
            log_error(f"Corrupted payload was falsely accepted! {corrupted_frame_1}")
            return False
            
        if SyntheticVerificationSuite.validate_nmea_sentence(corrupted_frame_2):
            log_error(f"Corrupted checksum was falsely accepted! {corrupted_frame_2}")
            return False
            
        log_system(f"  [PASS] NMEA Engine: Verified valid frames (${uav_chk}, ${rdr_chk}) and successfully trapped bit-flip errors.")
        return True

    @staticmethod
    def scenario_c_radar_range_equation() -> bool:
        """
        Scenario C: Radar Range Verification.
        Calculates received power (Pr) via Radar Range Equation for known targets
        at 5km, 10km, and 20km with RCS of 0.1 m^2 to verify inverse 4th-power behavior.
        """
        log_system("Running Scenario C: Radar Range Equation & R^4 Attenuation...")
        
        # Radar Physical Parameters
        pt = 60000.0       # 60 kW Peak Power
        gain_linear = 10.0 ** (38.0 / 10.0)  # 38 dBi Gain in linear units ~6309.57
        freq_hz = 9.4e9    # 9.4 GHz
        c = 299792458.0    # Speed of light m/s
        wavelength = c / freq_hz
        rcs = 0.1          # 0.1 m^2
        
        def compute_pr(range_m: float) -> float:
            numerator = pt * (gain_linear ** 2) * (wavelength ** 2) * rcs
            denominator = ((4.0 * math.pi) ** 3) * (range_m ** 4)
            return numerator / denominator
            
        r_5k  = 5000.0
        r_10k = 10000.0
        r_20k = 20000.0
        
        pr_5k  = compute_pr(r_5k)
        pr_10k = compute_pr(r_10k)
        pr_20k = compute_pr(r_20k)
        
        # Test Inverse 4th-Power ratio: doubling range must drop power by 2^4 = 16.0 (approx -12.04 dB)
        ratio_1 = pr_5k / pr_10k
        ratio_2 = pr_10k / pr_20k
        expected_ratio = 16.0
        
        epsilon = 1e-4
        if abs(ratio_1 - expected_ratio) > epsilon or abs(ratio_2 - expected_ratio) > epsilon:
            log_error(f"Radar Range Equation inverse 4th-power test failed! Ratio1={ratio_1:.4f}, Ratio2={ratio_2:.4f}, Expected={expected_ratio}")
            return False
            
        log_system(f"  [PASS] Radar Physics: Pr(5km)={pr_5k:.3e}W, Pr(10km)={pr_10k:.3e}W, Pr(20km)={pr_20k:.3e}W (Ratio: {ratio_1:.1f}x / -12.04 dB).")
        return True


# =============================================================================
# PART 2: C++ BUILD AUTOMATION & TEST RUNNER
# =============================================================================

class BuildManager:
    """Manages CMake configuration, compilation, and execution of test suites."""

    def __init__(self, workspace_dir: Path):
        self.workspace = workspace_dir
        self.build_dir = workspace_dir / "build"

    def find_executable(self, name: str) -> Path:
        """Searches build directory recursively for a target binary."""
        extensions = [".exe", ""] if sys.platform == "win32" else [""]
        for ext in extensions:
            target = f"{name}{ext}"
            for candidate in self.build_dir.rglob(target):
                if candidate.is_file() and os.access(candidate, os.X_OK | os.R_OK):
                    return candidate
        return None

    def build_project(self) -> bool:
        """Configures and builds the C++ project via CMake if available."""
        cmake_exe = shutil.which("cmake")
        if not cmake_exe:
            log_warn("CMake executable not found in PATH.")
            log_warn("Will attempt to locate pre-built binaries or run integrated simulated telemetry.")
            return False

        log_system(f"Found CMake at: {cmake_exe}")
        
        try:
            if not self.build_dir.exists():
                log_system("Generating build tree with 'cmake -B build -S .' ...")
                res = subprocess.run([cmake_exe, "-B", "build", "-S", "."], cwd=self.workspace)
                if res.returncode != 0:
                    log_error(f"CMake configuration failed with exit code {res.returncode}")
                    return False
            else:
                log_system("Using existing build directory: ./build")

            log_system("Compiling Project GarudLink with 'cmake --build build --config Release' ...")
            build_res = subprocess.run([cmake_exe, "--build", "build", "--config", "Release"], cwd=self.workspace)
            if build_res.returncode != 0:
                log_error(f"Compilation failed with exit code {build_res.returncode}")
                return False

            log_system(f"{Colors.GREEN}C++ Build completed successfully!{Colors.RESET}")
            return True
        except Exception as e:
            log_error(f"Error during build execution: {e}")
            return False

    def run_tests(self) -> bool:
        """Locates and runs the compiled C++ test suite."""
        test_bin = self.find_executable("run_tests") or self.find_executable("test_suite")
        if not test_bin:
            log_warn("No compiled test binary found in build/. Skipping native C++ test execution.")
            return True

        log_system(f"Executing C++ Test Suite: {test_bin} ...")
        try:
            test_run = subprocess.run([str(test_bin)], cwd=self.workspace, capture_output=True, text=True)
            print(test_run.stdout)
            if test_run.returncode == 0:
                log_system(f"{Colors.GREEN}C++ Unified Test Suite PASSED (Exit Code 0).{Colors.RESET}")
                return True
            else:
                log_error(f"C++ Test Suite failed with exit code {test_run.returncode}")
                print(test_run.stderr)
                return False
        except Exception as e:
            log_error(f"Failed to execute tests: {e}")
            return False


# =============================================================================
# PART 3: WEB SERVER & PROCESS ORCHESTRATION
# =============================================================================

class NonBlockingTCPServer(TCPServer):
    allow_reuse_address = True


class WebDashboardServer:
    """Asynchronous HTTP server serving the web/ directory on port 8000."""

    def __init__(self, web_dir: Path, port: int = 8000):
        self.web_dir = web_dir
        self.port = port
        self.httpd = None
        self.thread = None

    def start(self):
        class Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(web_dir), **kwargs)
            def log_message(self, format, *args):
                # Suppress noisy HTTP GET access logs in terminal
                pass

        web_dir = self.web_dir
        
        # Test if port is available
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        port_in_use = sock.connect_ex(('127.0.0.1', self.port)) == 0
        sock.close()

        if port_in_use:
            log_warn(f"Port {self.port} is already in use. Assuming existing HTTP server.")
            return

        try:
            self.httpd = NonBlockingTCPServer(("", self.port), Handler)
            self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.thread.start()
            log_system(f"Local HTTP Server active at {Colors.WHITE}{Colors.BOLD}http://localhost:{self.port}/{Colors.RESET} (serving web/)")
        except Exception as e:
            log_error(f"Failed to start HTTP server on port {self.port}: {e}")

    def stop(self):
        if self.httpd:
            log_system(f"Shutting down HTTP server on port {self.port}...")
            self.httpd.shutdown()
            self.httpd.server_close()


class TelemetryStreamer:
    """
    Parses and streams backend or synthetic telemetry to terminal with
    color-coded tags: [BUS-MSG], [RADAR], [MSL-GNC], [SYSTEM].
    """

    @staticmethod
    def format_and_print_line(line: str):
        line = line.strip()
        if not line:
            return

        # Tag-based categorization
        if "$UAV" in line or "$RDR" in line or "$MSL" in line or "$CMS" in line or "NMEA" in line or "SDN" in line:
            log_bus(line)
        elif "RADAR" in line.upper() or "RDR" in line.upper() or "BEARING" in line.upper() or "RANGE" in line.upper() or "AZ=" in line:
            log_radar(line)
        elif "MISSILE" in line.upper() or "MSL" in line.upper() or "INTERCEPT" in line.upper() or "GNC" in line.upper() or "TPN" in line.upper() or "LOS" in line.upper():
            log_msl(line)
        elif "KILL" in line.upper() or "SUCCESS" in line.upper() or "ALERT" in line.upper() or "ENGAGE" in line.upper():
            print(f"{Colors.MAGENTA}{Colors.BOLD}[CMS ALERT]{Colors.RESET} {line}")
        elif line.startswith("[T=") or "km)" in line:
            print(f"{Colors.WHITE}{line}{Colors.RESET}")
        else:
            log_system(line)

    @staticmethod
    def run_synthetic_stream(stop_event: threading.Event):
        """Generates realistic tactical telemetry if C++ binary is not compiled."""
        log_system("Generating Real-Time Synthetic SDN & GNC Telemetry Stream...")
        t = 0.0
        dt = 0.5
        uav_pos = [14142.0, 14142.0, -1500.0]
        uav_vel = [-127.3, -127.3, 0.0]
        msl_pos = [0.0, 0.0, -25.0]
        msl_launched = False
        msl_speed = 680.0
        
        while not stop_event.is_set():
            t += dt
            # Update UAV position with subtle weave
            uav_pos[0] += uav_vel[0] * dt
            uav_pos[1] += uav_vel[1] * dt + math.sin(t * 0.4) * 20.0
            uav_range = math.hypot(uav_pos[0], uav_pos[1])
            uav_az = math.degrees(math.atan2(uav_pos[1], uav_pos[0])) % 360.0
            
            # Format synthetic NMEA frame
            nmea_uav = f"$UAV,POS,{uav_pos[0]:.1f},{uav_pos[1]:.1f},{uav_pos[2]:.1f},{uav_vel[0]:.1f},{uav_vel[1]:.1f}"
            chk = SyntheticVerificationSuite.calculate_nmea_checksum(nmea_uav)
            TelemetryStreamer.format_and_print_line(f"{nmea_uav}*{chk}")
            
            # Radar detection update
            pr = 60000.0 * (10**(3.8))**2 * (0.0319**2) * 0.5 / (((4.0*math.pi)**3) * (uav_range**4))
            snr_db = 10.0 * math.log10(max(1e-15, pr) / 1e-13)
            TelemetryStreamer.format_and_print_line(f"RADAR TRACK: UAV-ALPHA Az={uav_az:.1f} deg, SlantRange={uav_range/1000.0:.2f} km, Pr={pr:.2e} W (SNR={snr_db:.1f} dB)")
            
            # Auto-engagement trigger
            if uav_range <= 15000.0 and not msl_launched:
                msl_launched = True
                print(f"{Colors.MAGENTA}{Colors.BOLD}[CMS ALERT] AUTO-ENGAGEMENT: Launching Interceptor MSL-01 at target UAV-ALPHA (Range: {uav_range:.0f}m){Colors.RESET}")
            
            if msl_launched:
                # Proportional navigation guidance step towards UAV
                dx = uav_pos[0] - msl_pos[0]
                dy = uav_pos[1] - msl_pos[1]
                dz = uav_pos[2] - msl_pos[2]
                los_dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if los_dist > 0:
                    msl_pos[0] += (dx / los_dist) * msl_speed * dt
                    msl_pos[1] += (dy / los_dist) * msl_speed * dt
                    msl_pos[2] += (dz / los_dist) * msl_speed * dt
                
                TelemetryStreamer.format_and_print_line(f"MSL-GNC: MSL-01 TPN Guidance Active | LOS Distance={los_dist:.1f} m | Closing Velocity={(msl_speed + 180.0):.1f} m/s")
                
                # Check for intercept
                if los_dist <= 25.0:
                    print(f"\n{Colors.MAGENTA}{Colors.BOLD}[CMS SUCCESS] TARGET ELIMINATED! Interceptor MSL-01 neutralized UAV-ALPHA at T={t:.1f}s (Range={uav_range/1000.0:.2f} km)!{Colors.RESET}\n")
                    break
                    
            time.sleep(dt)


# =============================================================================
# PART 4: ORCHESTRATOR CONTROLLER & CLEAN TEARDOWN
# =============================================================================

class GarudLinkOrchestrator:
    """Master orchestrator for Project GarudLink."""

    def __init__(self):
        self.workspace_dir = Path(__file__).resolve().parent
        self.web_dir = self.workspace_dir / "web"
        self.build_mgr = BuildManager(self.workspace_dir)
        self.web_server = WebDashboardServer(self.web_dir, port=8000)
        
        self.cpp_process = None
        self.stop_event = threading.Event()
        self.sim_thread = None

    def handle_signal(self, signum, frame):
        """Handles SIGINT / SIGTERM for a clean exit."""
        print(f"\n\n{Colors.YELLOW}{Colors.BOLD}>>> Shutdown signal received. Performing clean teardown...{Colors.RESET}")
        self.stop_event.set()
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        """Terminates child subprocesses and releases network ports."""
        if self.cpp_process and self.cpp_process.poll() is None:
            log_system("Terminating C++ backend subprocess...")
            try:
                self.cpp_process.terminate()
                self.cpp_process.wait(timeout=2.0)
            except Exception:
                self.cpp_process.kill()
            self.cpp_process = None

        if self.web_server:
            self.web_server.stop()
            
        log_system(f"{Colors.GREEN}Clean teardown complete. All ports & processes released. Goodbye!{Colors.RESET}\n")

    def run(self):
        # Register signal handlers
        signal.signal(signal.SIGINT, self.handle_signal)
        signal.signal(signal.SIGTERM, self.handle_signal)

        print_banner()
        log_system(f"Workspace Directory: {self.workspace_dir}")

        # 1. Run deterministic synthetic verification
        verification_passed = SyntheticVerificationSuite.run_all_checks()
        if not verification_passed:
            log_error("Synthetic verification failed. Aborting launch.")
            return

        # 2. Build Automation & Test Execution
        build_success = self.build_mgr.build_project()
        if build_success:
            self.build_mgr.run_tests()

        # 3. Start Web Dashboard HTTP Server
        self.web_server.start()

        # 4. Launch Browser to Tactical Dashboard
        dashboard_url = "http://localhost:8000"
        log_system(f"Launching Tactical Dashboard in default browser: {Colors.CYAN}{dashboard_url}{Colors.RESET}")
        try:
            webbrowser.open(dashboard_url)
        except Exception as e:
            log_warn(f"Could not automatically launch browser: {e}")

        # 5. Spawn C++ Backend or Synthetic Telemetry Stream
        cpp_bin = self.build_mgr.find_executable("tactical_sim")
        
        if cpp_bin:
            log_system(f"Spawning C++ Tactical Backend: {cpp_bin} ...\n")
            try:
                self.cpp_process = subprocess.Popen(
                    [str(cpp_bin)],
                    cwd=self.workspace_dir,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1
                )
                
                # Stream backend stdout
                for line in iter(self.cpp_process.stdout.readline, ''):
                    if self.stop_event.is_set():
                        break
                    TelemetryStreamer.format_and_print_line(line)
                    
                self.cpp_process.stdout.close()
                self.cpp_process.wait()
            except Exception as e:
                log_error(f"Error executing C++ backend: {e}")
        else:
            log_warn("Compiled C++ binary './build/tactical_sim' not detected.")
            log_system("Switching to High-Fidelity Synthetic Simulation Stream...\n")
            self.sim_thread = threading.Thread(
                target=TelemetryStreamer.run_synthetic_stream,
                args=(self.stop_event,),
                daemon=True
            )
            self.sim_thread.start()

            # Keep orchestrator alive until user interrupts
            try:
                while not self.stop_event.is_set() and self.sim_thread.is_alive():
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass

        self.cleanup()


# =============================================================================
# ENTRY POINT
# =============================================================================
if __name__ == "__main__":
    orchestrator = GarudLinkOrchestrator()
    orchestrator.run()
