# bot.py
import os

import discord
import argparse
import json

# arg parse issue assignees
parser = argparse.ArgumentParser()
parser.add_argument("--issue")
args = parser.parse_args()
assignees = json.loads(args.issue)

names = [assignee['login'] for assignee in assignees]


TOKEN = os.getenv("DISCORD_TOKEN")

intents = discord.Intents.default()
client = discord.Client(intents=intents)


@client.event
async def on_ready():
    print(f'{client.user} has connected to Discord!')
    await send_message()


async def send_message():
    for guild in client.guilds:
        if guild.name == "McGill Robotics":
            for channel in guild.channels:
                if channel.name == "discord-support":
                    await channel.send("Test, an issue has been closed, assingees data is " + str(assignees))



client.run(TOKEN)