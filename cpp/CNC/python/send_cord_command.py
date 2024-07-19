
import const
import networktables
import sys

def CommandGoToLocation(CNCTable: networktables.NetworkTable, x, y):
    glocTable = CNCTable.getSubTable("GoToLocation")
    glocTable.putNumber("x", float(x))
    glocTable.putNumber("y", float(y))


if __name__ == "__main__":
    networktables.NetworkTables.initialize(const.NT_IP)
    CNCTable = networktables.NetworkTables.getTable("SmartDashboard").getSubTable(const.CNC_TABLE_NAME)

    print("send_cord_command in tty mode, press q or ctrl-c + enter to exit")
    while True:
        try:
            print("Please enter the target cordinates, space seperated list of float, (x, y):")
            cmd = input().strip()
            if cmd == "q":
                print("exiting")
                exit(0)

            # string processing
            cordArr = cmd.split()
            x = float(cordArr[0])
            y = float(cordArr[1])

            # x,y upper and lower bound checking 
            if not (const.FIELD_SIZE_LIMIT[0][0] <= x < const.FIELD_SIZE_LIMIT[0][1]): 
                print("ERROR: x cord out of bounds")
                continue
            if not (const.FIELD_SIZE_LIMIT[1][0] < y < const.FIELD_SIZE_LIMIT[1][1]):
                print("ERROR: y cord out of bounds")
            
            print(f"Sending GoToLocation ({x}, {y}) to robot")
            CommandGoToLocation(CNCTable, x, y)

        except KeyboardInterrupt:exit(0)
        except SystemExit:exit(0)
        except:
            print("ERROR, please enter again.")
            continue
