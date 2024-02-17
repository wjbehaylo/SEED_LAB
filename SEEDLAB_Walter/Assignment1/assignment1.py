import numpy as np

#exercise 1

def get_stats(data_set): #pass in the data_set and output the desired characteristics
    maximum = max(data_set) #use the built in function max to determine the maximum of the list
    print(f"The maximum value is: {maximum}") #print statement for output
    minimum = min(data_set) #use built in function min to get minimum of list
    print(f"The minimum value is: {minimum}")
    index = data_set.index(38) #use built in function index to get index of desired value (38 here)
    print(f"The index of 38 is {index}")
    #for getting most repeated numbers, and their count, it is less easy
    #first, we find the mode by making elements unique and finding which has the largest count in the data_set.
    #to do this, I looked up an example, then used the key option of max and the .count member of the list datatype. This makes it so that we are getting the elements and returning which occurs the most
    mode = max(set(data_set), key=data_set.count) 
    #then, we can access the count of that element
    count = data_set.count(mode)
    print(f"The most commonly occuring element is {mode}, with a count of {count}") #then we output it
    
    data_set_array = np.array(data_set) #to be honest I didn't look up documentation for this conversion and just guessed correctly, then checked afterwards
    data_set_array.sort() # I know that numpy arrays have a built in .sort() method (also can only store one data type)
    print(f"The list sorted is {data_set_array}")
    
    #I looked up in chatgpt how to do list comprehension, then tried to implement it here. I don't think this counts as a for loop
    data_set_even = [x for x in data_set_array if x%2 == 0] #I know simple conditional to check if int is even
    print(f"The even numbers in order are {data_set_even}")
    
    
with open('datafile.txt', 'r') as f:
    data = eval(f.read())
    get_stats(data)

#exercise 2

# my plan is to first define functions for each state, where they will be called upon new input.
# the new input will be input to the function for the state, and based on the state's function and input we will transition states
# to get these states, I just followed the state diagram
def state0(character):
    if(character == 'a'):
        return stateA
    else:
        return state0
    
def stateA(character):
    if(character =='b'): 
        return stateB
    elif(character =='a'):
        return stateA
    else:
        return state0
    
def stateB(character):
    if(character =='c'):
        return stateC
    elif(character =='a'):
        return stateA
    else:
        return state0
    
    
def stateC(character):
    if(character =='d'):
        return stateD
    elif(character =='a'):
        return stateA
    else:
        return state0
    
def stateD(character):
    print("abcd is contained in the string")
    #I'm not sure if the program should end here. So I commented out these following lines 
    '''
    if(character == 'a'):
        return stateA
    else: 
        return state0
    '''
    return stateFinished
    
def stateFinished():
    #this is the final state, we go to this after stateD, and don't do anything from here
    return 0

state_dictionary = { #making the state dictionary for my code. 
    state0 : "0",
    stateA : "a",
    stateB : "b",
    stateC : "c",
    stateD : "d",
    stateFinished : "Finished"
}
state = state0
input_string =input("Enter the string\n")
index = 0 #first character in the string we are checking.
while state is not stateFinished and index != len(input_string): #run until we have the characters or have a full string
    #output current state
    print(f"Current state is {state_dictionary[state]}") #access '0', 'a', 'b', 'c', or 'd' respectively
    
    #access the next character of the input string
    character = input_string[index]
    
    #increment the input string character so that next we do the next one
    index += 1
    
    #basically we declare new_state to execute the state currently in function through a pointer, and assign it to be the next state
    new_state = state(character) 
    state = new_state #now we update to be the next state
print("Done with state machine") #print this when the execution is completed.
