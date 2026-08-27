#!/usr/bin/env python3
import os
import discord
import threading
import asyncio
from discord.ext import commands
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dotenv import load_dotenv

load_dotenv()

class DiscordMultiModeBotNode(Node):
    def __init__(self):
        super().__init__('discord_multimode_node')

        intents = discord.Intents.default()
        intents.message_content = True
        self.bot = commands.Bot(command_prefix="!", intents=intents)

        # === Paths — must match perimeter_dir used by path_perimeter_node,
        # path_generator_node, and stanlynavigation_node in bunker_navigation
        self.declare_parameter('perimeter_dir', '/home/bidya/Kevares-Code-v2/data/perimeter')
        self.perimeter_dir = self.get_parameter('perimeter_dir').get_parameter_value().string_value

        # === State
        self.perimeter_name = None
        self.path_name = None
        self.discord_loop = None
        self.last_alert = None

        # === ROS publishers
        # Perimeter & navigation (existing)
        self.perimeter_cmd_pub    = self.create_publisher(String, '/perimeter_cmd', 10)
        self.record_cmd_pub       = self.create_publisher(String, '/record_cmd', 10)
        self.perimeter_name_pub   = self.create_publisher(String, '/perimeter_name', 10)
        self.path_name_pub        = self.create_publisher(String, '/path_name', 10)
        self.path_gen_trigger_pub = self.create_publisher(String, '/generate_path_trigger', 10)   # NO obstacles
        self.nav_trigger_pub      = self.create_publisher(String, '/stanley_start_cmd', 10)
        self.cargo_nav_trigger_pub = self.create_publisher(String, '/cargo_start_cmd', 10)
        self.gps_log_cmd_pub       = self.create_publisher(String, '/gps_log_cmd', 10)

        # Obstacle recording
        self.obstacle_cmd_pub     = self.create_publisher(String, '/obstacle_cmd', 10)       # "start"/"stop"
        self.obstacle_id_pub      = self.create_publisher(String, '/obstacle_id', 10)        # "1","2",...
        self.obstacles_done_pub   = self.create_publisher(String, '/obstacles_finish', 10)   # "finish"

        # With-obstacles path generator trigger (separate node)
        self.path_gen_withobs_pub = self.create_publisher(String, '/generate_path_withobs', 10)

        self.setup_bot_commands()

    # ----------------- ROS subscriptions -----------------
    def setup_ros_subscriptions(self):
        self.create_subscription(String, '/obstacle_alert', self.alert_callback, 10)

    def alert_callback(self, msg):
        new_alert = msg.data.strip()
        if not new_alert or new_alert == self.last_alert:
            return

        async def send_alert():
            await self.bot.wait_until_ready()
            if new_alert == "speed_zero":
                message = "Obstacle detected. Robot stopped!"
            elif new_alert == "speed_normal":
                message = "Robot is moving now."
            else:
                message = "Robot Alert: " + new_alert

            for guild in self.bot.guilds:
                for channel in guild.text_channels:
                    if channel.permissions_for(guild.me).send_messages:
                        await channel.send(message)
                        break

        if self.discord_loop:
            asyncio.run_coroutine_threadsafe(send_alert(), self.discord_loop)
        self.last_alert = new_alert

    # ----------------- ROS helpers -----------------
    def _pub_str(self, pub, text):
        msg = String(); msg.data = text; pub.publish(msg)

    def send_perimeter_cmd(self, c):     self._pub_str(self.perimeter_cmd_pub, c)
    def send_perimeter_name(self, n):    self._pub_str(self.perimeter_name_pub, n)
    def send_record_cmd(self, c):        self._pub_str(self.record_cmd_pub, c)
    def send_path_name(self, n):         self._pub_str(self.path_name_pub, n)
    def send_nav(self, action):          self._pub_str(self.nav_trigger_pub, action)
    def send_cargo_nav(self, action):    self._pub_str(self.cargo_nav_trigger_pub, action)
    def send_gps_log(self, action):      self._pub_str(self.gps_log_cmd_pub, action)

    # NO-obstacles generator
    def send_path_gen_trigger(self):     self._pub_str(self.path_gen_trigger_pub, "start")   # /generate_path_trigger

    # Obstacle helpers
    def send_obstacle_cmd(self, c):      self._pub_str(self.obstacle_cmd_pub, c)     # "start"/"stop"
    def send_obstacle_id(self, oid):     self._pub_str(self.obstacle_id_pub, oid)    # "1","2",...
    def send_obstacles_done(self):       self._pub_str(self.obstacles_done_pub, "finish")

    # With-obstacles generator
    def send_path_gen_withobs(self):     self._pub_str(self.path_gen_withobs_pub, "start")   # /generate_path_withobs

    # ----------------- Bot commands -----------------
    def setup_bot_commands(self):
        @self.bot.event
        async def on_ready():
            self.discord_loop = asyncio.get_running_loop()
            self.setup_ros_subscriptions()

            async def prompt_mode(channel):
                await channel.send(f"Bot online as {self.bot.user}.")
                await channel.send("What do you want to do? Type **lawn** or **cargo**.")

                def check(m):
                    return not m.author.bot and m.channel.id == channel.id and m.content.strip().lower() in ("lawn", "cargo")

                try:
                    resp = await self.bot.wait_for('message', check=check, timeout=60)
                    choice = resp.content.strip().lower()
                    if choice == "lawn":
                        await channel.send("Lawn mowing selected. Use **!drawperimeter** to begin.")
                        await channel.send("When perimeter is done, use **!obstacles** to record obstacle polygons.")
                        await channel.send("Generate with **!generate** → choose **with** or **without** obstacles.")
                    else:
                        await channel.send("Cargo hauling selected.\nUse **!cargo_record** to record and **!cargo_nav** to follow.")
                except asyncio.TimeoutError:
                    await channel.send("No response. Use **!start** later to choose lawn/cargo.")

            for guild in self.bot.guilds:
                for channel in guild.text_channels:
                    if channel.permissions_for(guild.me).send_messages:
                        asyncio.create_task(prompt_mode(channel))
                        break

        # ---------- Manual start ----------
        @self.bot.command()
        async def start(ctx):
            await ctx.send("What do you want to do? Type **lawn** or **cargo**.")

            def check(m): return m.author == ctx.author and m.channel == ctx.channel
            try:
                resp = await self.bot.wait_for('message', check=check, timeout=60)
                choice = resp.content.strip().lower()
                if choice == "lawn":
                    await ctx.send("Lawn mowing selected. Use **!drawperimeter** to begin.")
                    await ctx.send("When perimeter is done, use **!obstacles** to record obstacle polygons.")
                    await ctx.send("Then **!generate** and choose **with** or **without** obstacles.")
                elif choice == "cargo":
                    await ctx.send("Cargo hauling selected. Use **!cargo_record** to record and **!cargo_nav** to follow.")
                else:
                    await ctx.send("Invalid choice. Use **!start** again.")
            except asyncio.TimeoutError:
                await ctx.send("Timeout. Use **!start** again.")

        # ---------- Lawn mowing ----------
        @self.bot.command()
        async def drawperimeter(ctx):
            await ctx.send("Name of perimeter?")
            def check(m): return m.author == ctx.author and m.channel == ctx.channel
            try:
                resp = await self.bot.wait_for('message', check=check, timeout=60)
                self.perimeter_name = resp.content.strip()
                self.send_perimeter_name(self.perimeter_name)
                os.makedirs(os.path.join(self.perimeter_dir, self.perimeter_name), exist_ok=True)
                await ctx.send(f"Perimeter set to **{self.perimeter_name}**. Use !send_y, !send_n, !send_e.")
                await ctx.send("After finishing the outer perimeter, use **!obstacles** to record obstacles before generating paths.")
            except asyncio.TimeoutError:
                await ctx.send("Timeout! Use **!drawperimeter** again.")

        @self.bot.command()
        async def send_y(ctx):
            self.send_perimeter_cmd('y')
            await ctx.send("Recording perimeter START.")

        @self.bot.command()
        async def send_n(ctx):
            self.send_perimeter_cmd('n')
            await ctx.send("Recording perimeter STOP.")

        @self.bot.command()
        async def send_e(ctx):
            self.send_perimeter_cmd('e')
            await ctx.send("Exit perimeter session.")

        # ---------- Obstacles workflow ----------
        @self.bot.command()
        async def obstacles(ctx):
            if not self.perimeter_name:
                await ctx.send("Please set a perimeter first with **!drawperimeter**.")
                return

            await ctx.send(
                "**Obstacle menu**:\n"
                "• Type **record obstacle?** to start/stop an obstacle polygon.\n"
                "• Type **finish obstacle** when ALL obstacles are recorded."
            )

            def top_check(m):
                return (m.author == ctx.author and m.channel == ctx.channel and
                        m.content.strip().lower() in ("record obstacle?", "finish obstacle"))

            try:
                choice_msg = await self.bot.wait_for('message', check=top_check, timeout=120)
                choice = choice_msg.content.strip().lower()

                if choice == "record obstacle?":
                    await ctx.send("Enter obstacle **ID** (e.g., 1, 2, 3 ...):")

                    def id_check(m):
                        return (m.author == ctx.author and m.channel == ctx.channel and m.content.strip().isdigit())

                    try:
                        id_msg = await self.bot.wait_for('message', check=id_check, timeout=60)
                        oid = id_msg.content.strip()
                        self.send_obstacle_id(oid)
                        await ctx.send(
                            f"Obstacle ID set to **{oid}**.\n"
                            "Type **start** to begin recording or **stop** to end this obstacle polygon."
                        )

                        def rec_check(m):
                            return (m.author == ctx.author and m.channel == ctx.channel and
                                    m.content.strip().lower() in ("start", "stop"))

                        recording_open = True
                        while recording_open:
                            try:
                                rec_msg = await self.bot.wait_for('message', check=rec_check, timeout=180)
                                cmd = rec_msg.content.strip().lower()
                                if cmd == "start":
                                    self.send_obstacle_cmd("start")
                                    await ctx.send("Obstacle recording **START**.")
                                else:
                                    self.send_obstacle_cmd("stop")
                                    await ctx.send("Obstacle recording **STOP**. Use **!obstacles** again to add another, or **finish obstacle** when done.")
                                    recording_open = False
                            except asyncio.TimeoutError:
                                await ctx.send("Timeout while waiting for **start/stop**. Use **!obstacles** to try again.")
                                recording_open = False

                    except asyncio.TimeoutError:
                        await ctx.send("Timeout waiting for obstacle **ID**. Use **!obstacles** again.")

                elif choice == "finish obstacle":
                    self.send_obstacles_done()
                    await ctx.send("Obstacles finished. You can now run **!generate** and choose **with** obstacles.")
            except asyncio.TimeoutError:
                await ctx.send("No selection. Use **!obstacles** again when ready.")

        @self.bot.command()
        async def obstacle_start(ctx, oid: str):
            if not oid.isdigit():
                await ctx.send("Provide a numeric obstacle ID, e.g., `!obstacle_start 3`.")
                return
            self.send_obstacle_id(oid)
            self.send_obstacle_cmd("start")
            await ctx.send(f"Obstacle **{oid}** recording START.")

        @self.bot.command()
        async def obstacle_stop(ctx):
            self.send_obstacle_cmd("stop")
            await ctx.send("Obstacle recording STOP.")

        @self.bot.command()
        async def obstacles_finish(ctx):
            self.send_obstacles_done()
            await ctx.send("Obstacles finished. You can now run **!generate** and choose **with**.")

        # ---------- SINGLE CHOOSER: Generate path (with/without) ----------
        @self.bot.command()
        async def generate(ctx):
            if not self.perimeter_name:
                await ctx.send("Set a perimeter first with **!drawperimeter**.")
                return

            await ctx.send("Generate path: type **without** (no obstacles) or **with** (use obstacles).")

            def check(m):
                return (m.author == ctx.author and m.channel == ctx.channel and
                        m.content.strip().lower() in ("without", "with"))

            try:
                resp = await self.bot.wait_for('message', check=check, timeout=60)
                choice = resp.content.strip().lower()

                if choice == "without":
                    self.send_path_gen_trigger()
                    await ctx.send("Generating path **without obstacles**...")
                else:
                    self.send_path_gen_withobs()
                    await ctx.send("Generating path **with obstacles**...")

                await asyncio.sleep(5)

                folder = os.path.join(self.perimeter_dir, self.perimeter_name or "")

                for f in [
                    "coverage_paths.png",
                    "coverage_paths_withobs.png",
                    "perimeter_with_obstacles.png",
                    "obstacles_plot.png",
                    "gps_polygon_plot.png",
                ]:
                    fp = os.path.join(folder, f)
                    if os.path.exists(fp):
                        try:
                            await ctx.send(file=discord.File(fp))
                        except Exception:
                            pass

                waypoint_names = []
                for w in ["waypoints_noobs.csv", "waypoints_withobs.csv"]:
                    if os.path.exists(os.path.join(folder, w)):
                        waypoint_names.append(w)
                if waypoint_names:
                    await ctx.send("Waypoints ready: " + ", ".join(waypoint_names))

                await ctx.send("Path generation done. Use **!confirm_start** to navigate.")
            except asyncio.TimeoutError:
                await ctx.send("Timeout. Use **!generate** again.")

        # ---------- Direct one-liners (kept for compatibility) ----------
        @self.bot.command()
        async def generate_path(ctx):
            self.send_path_gen_trigger()
            await ctx.send("Generating zigzag paths (no obstacles)...")
            await asyncio.sleep(5)
            folder = os.path.join(self.perimeter_dir, self.perimeter_name or "")
            for f in ["gps_polygon_plot.png", "coverage_paths.png"]:
                fp = os.path.join(folder, f)
                if os.path.exists(fp):
                    try:
                        await ctx.send(file=discord.File(fp))
                    except Exception:
                        pass
            await ctx.send("Path generation (no obstacles) done. Use !confirm_start to navigate.")

        @self.bot.command(aliases=["gen_withobs"])
        async def generate_path_withobs(ctx):
            if not self.perimeter_name:
                await ctx.send("Set a perimeter first with **!drawperimeter**.")
                return
            self.send_path_gen_withobs()
            await ctx.send("Generating zigzag paths **with obstacles**...")
            await asyncio.sleep(5)
            folder = os.path.join(self.perimeter_dir, self.perimeter_name or "")
            for f in ["perimeter_with_obstacles.png", "obstacles_plot.png", "coverage_paths_withobs.png", "gps_polygon_plot.png"]:
                fp = os.path.join(folder, f)
                if os.path.exists(fp):
                    try:
                        await ctx.send(file=discord.File(fp))
                    except Exception:
                        pass
            wp_csv = os.path.join(folder, "waypoints_withobs.csv")
            if os.path.exists(wp_csv):
                await ctx.send("Waypoints (with obstacles) saved: waypoints_withobs.csv")
            await ctx.send("Path generation (with obstacles) done. Use !confirm_start to navigate.")

        # ---------- Start/Stop navigation ----------
        @self.bot.command()
        async def confirm_start(ctx):
            await ctx.send("Which perimeter to navigate?")
            def check(m): return m.author == ctx.author and m.channel == ctx.channel
            try:
                resp = await self.bot.wait_for('message', check=check, timeout=60)
                chosen = resp.content.strip()
                if not os.path.exists(os.path.join(self.perimeter_dir, chosen)):
                    await ctx.send("Perimeter not found."); return
                self.perimeter_name = chosen
                self.send_perimeter_name(chosen)
                self.send_nav("start")
                await ctx.send(f"Navigating perimeter **{chosen}**.")
            except asyncio.TimeoutError:
                await ctx.send("Timeout. Use **!confirm_start** again.")

        @self.bot.command()
        async def stop_navigation(ctx):
            self.send_nav("stop")
            await ctx.send("Navigation stopped.")

        # ---------- Cargo hauling ----------
        @self.bot.command()
        async def cargo_record(ctx):
            await ctx.send("Name for cargo path?")
            def check(m): return m.author == ctx.author and m.channel == ctx.channel
            try:
                resp = await self.bot.wait_for('message', check=check, timeout=60)
                self.perimeter_name = resp.content.strip()
                self.send_perimeter_name(self.perimeter_name)
                os.makedirs(os.path.join(self.perimeter_dir, self.perimeter_name), exist_ok=True)
                await ctx.send(f"Cargo path set to **{self.perimeter_name}**. Use !cargo_rec_start, !cargo_rec_stop.")
            except asyncio.TimeoutError:
                await ctx.send("Timeout! Use **!cargo_record** again.")

        @self.bot.command()
        async def cargo_rec_start(ctx):
            self.send_record_cmd("start")
            await ctx.send("Cargo path recording START.")

        @self.bot.command()
        async def cargo_rec_stop(ctx):
            self.send_record_cmd("stop")
            await asyncio.sleep(1)
            folder = os.path.join(self.perimeter_dir, self.perimeter_name or "")
            await ctx.send(f"Cargo path STOP. Data in `{folder}`.")
            await ctx.send("Use **!cargo_nav** to follow this path directly.")

        @self.bot.command()
        async def cargo_nav(ctx):
            await ctx.send("Which cargo path to follow?")
            def check(m): return m.author == ctx.author and m.channel == ctx.channel
            try:
                resp = await self.bot.wait_for('message', check=check, timeout=60)
                chosen = resp.content.strip()
                if not os.path.exists(os.path.join(self.perimeter_dir, chosen)):
                    await ctx.send("Path not found."); return
                self.perimeter_name = chosen
                self.send_perimeter_name(chosen)
                self.send_nav("start")
                await ctx.send(f"Navigating cargo path **{chosen}**. Use **!cargo_stop** to stop.")
            except asyncio.TimeoutError:
                await ctx.send("Timeout. Use **!cargo_nav** again.")

        @self.bot.command()
        async def cargo_stop(ctx):
            self.send_nav("stop")
            await ctx.send("Cargo navigation stopped.")

    # ----------------- Run bot -----------------
    def run(self, token):
        self.bot.run(token)

def main(args=None):
    rclpy.init(args=args)
    node = DiscordMultiModeBotNode()
    ros_thread = None
    try:
        token = os.getenv("DISCORD_BOT_TOKEN", "").strip()
        if not token:
            node.get_logger().error("DISCORD_BOT_TOKEN env var not set. Export it before running.")
            return
        ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        ros_thread.start()
        node.run(token)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if ros_thread is not None:
            ros_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()