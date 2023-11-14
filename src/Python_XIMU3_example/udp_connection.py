import connection
import helpers
import ximu3

if helpers.ask_question("Search for connections?"):
    print("Searching for connections")

    messages = ximu3.NetworkAnnouncement().get_messages_after_short_delay()

    if not messages:
        raise Exception("No UDP connections available")

    print("Found " + messages[0].device_name + " " + messages[0].serial_number)

    connection.run(messages[0].to_udp_connection_info())
else:
    connection.run(ximu3.UdpConnectionInfo("0.0.0.0", 9000, 7000))
