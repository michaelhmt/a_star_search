print("This program calculates the air conditioning window cooling capacity of a ventilation unit.\n")

while True:
    roomName = input("Enter the name of the room: ")
    try:
        length = int(input("\nEnter the length of the room in 5 feet or more: \n\n"))
        print(f"\n\nThe length of the room is {length} feet.")
        if length < 5:
            raise ValueError
        width = int(input("\nEnter the width of the room in 5 feet or more: \n\n"))
        print(f"\nThe width of the room is {width} feet.")
        if width < 5:
            raise ValueError
        area = length * width
        print(f"\nThe area of the room is {area} square feet.")
        room_Size = length * width
        break # Exit the loop if both length and width are valid
    except ValueError:
        print("\nInvalid input. Please enter a valid input for length and width greater than 5 feet.\n")

    print("\nWhat is the amount of shade that this room receives?\n")
    print("1. Little Shade")
    print("2. Moderate Shade")
    print("3. Abundant Shade")

    user_input = int(input("\nPlease pick one of the options above: "))
    print("Air Conditioning Window Unit Cooling Capacity \n")
    print("Room name:", roomName)

    if user_input == 1:
        print("Amount of shade: Little")
        ac_capacity = 5500 + (room_Size * 0.15)
    elif user_input == 2:
        print("Amount of shade: Moderate")
        ac_capacity = 5500 + (room_Size * 0.15)
    elif user_input == 3:
        print("Amount of shade: Abundant")
        ac_capacity = 5500 + (room_Size * 0.15)

    if user_input in [1, 2, 3]:
        print("BTU's needed per hour:", ac_capacity)
        print("Room area (in square feet):", room_Size)
    else:
        raise ValueError("Invalid input. Please select a shade level from 1 to 3.")

    while True:
        user_input = input("Do you want to continue? (yes/no): ")
        if user_input.lower() in ["yes", "y"]:
            print("Continuing...")
            break
        elif user_input.lower() in ["no", "n"]:
            print("Exiting...")
            break
        else:
            print("Invalid input. Please enter yes/no.")