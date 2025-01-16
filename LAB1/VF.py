# receives image
import math
import numpy as np
import cv2 as cv
import skimage.morphology
from PIL import Image
import serial
import time
import datetime
import subprocess
import tempfile
import re

#______________________ IMAGE_PROCESSING FUNCTIONS ________________________#
#-----------------------------------------------------------------------#
#Name: distance()
#Description: calculates the distance between two points
#Parameters: x1 - x coordinate of the first point 
#            x2 - x coordinate of the second point
#            y1 - y coordinate of the first point
#            y2 - y coordinate of the second point
#Return: distance
#-----------------------------------------------------------------------#
def distance(x1,x2,y1,y2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

#-----------------------------------------------------------------------#
#Name: get_mask2()
#Description: creates a mask for the contours taking into account the 
#             distance the point and the last point that was added to 
#             the mask
#Parameters: array - array of contours
#            threshold - threshold for the distance between points
#Return: mask
#----------------------------------------------------------------------#
def get_mask2(array, threshold):
    mask = np.zeros(len(array), dtype=bool)
    last_point = 0
    mask[0] = True
    for i in range(len(array)):
        if distance(array[last_point][0], array[i][0], array[last_point][1], array[i][1]) > threshold:
            mask[i] = True
            last_point = i
    mask[-1] = True
    mask[-2] = True
    mask[-3] = True
    return mask

#-----------------------------------------------------------------------#
#Name: get_mask()
#Description: creates a mask for the contours taking into account the
#             distance the point and the last point searched
#Parameters: array - array of contours
#            threshold - threshold for the distance between points
#Return: mask
#-----------------------------------------------------------------------#
def get_mask(array, threshold):
    mask = np.zeros(len(array), dtype=bool)
    last_point = 0
    for i in range(len(array)):
        if distance(array[last_point][0], array[i][0], array[last_point][1], array[i][1]) > threshold:
            mask[i] = True
        last_point = i
    return mask

#-----------------------------------------------------------------------#
#Name: get_max_idx()
#Description: gets the index of the n largest element in a list, where n
#            is the number of iterations
#Parameters: lista - list of elements
#            number_of_iter - number of iterations
#Return: index of the largest element
#-----------------------------------------------------------------------#
def get_max_idx (lista, number_of_iter):

    lists =[]

    lists = sorted(enumerate(lista), key=lambda x: len(x[1]), reverse=True)

    cnt_largest_idx = lists[number_of_iter-1][0]

    return cnt_largest_idx


#-----------------------------------------------------------------------#
#Name: get_new_contours()
#Description: copy largest contour to the first position of the list and
#             the rest of the contours to the last position of the list
#             and then it removes the points that are repeated or unwanted
#Parameters: my_list - list of contours
#Return: new list of contours
#-----------------------------------------------------------------------#
def get_new_contours (my_list):

    side_list = []

    iter_value = 1

    side_list = my_list.copy()

    threshold_1 = 2

    max_list = []
    min_list = []

    for list1 in my_list:

        my_idx = get_max_idx(side_list, iter_value)

        if iter_value == 1:

            max_list = np.copy(my_list[my_idx])

            iter_value +=1

        else:

            for i in range(len(my_list[my_idx])):
                repeated = False
                for j in range(len(max_list)):
                    if ((((my_list[my_idx][i][0][0] == max_list[j][0][0])) & 
                    ((my_list[my_idx][i][0][1] == max_list[j][0][1]))) or 
                    distance(my_list[my_idx][i][0][0], max_list[j][0][0], 
                    my_list[my_idx][i][0][1], max_list[j][0][1]) < threshold_1):
                        repeated = True
                if not repeated:
                    max_list = np.append(max_list,[my_list[my_idx][i]],axis=0)
                    if (len(min_list)==0):
                        min_list = np.copy([my_list[my_idx][i]])
                    else:
                        min_list = np.append(min_list,[my_list[my_idx][i]],axis=0)
                    
                    
            my_list[my_idx] = np.copy(min_list)

            min_list = []

            iter_value +=1  
            
    return my_list

#-----------------------------------------------------------------------#
#Name: reduce_dim()
#Description: Conversion of our array to a 2D array
#Parameters: array - array of contours
#Return: new array
#-----------------------------------------------------------------------#
def reduce_dim(array):

    new_array=[]
    for i in range(len(array)):
        new_array.append(np.reshape(array[i], (array[i].shape[0],2)))

    return new_array

#-----------------------------------------------------------------------#
#Name: connected()
#Description: conects the contours
#Parameters: lista - list of contours
#            mask - mask of the contours
#            k - index of the first contour
#Return: new list of contours
#-----------------------------------------------------------------------#
def connected(lista,mask,k):
    for i in range(len(mask)):
        if i == 0:
            lista.pop(k)
            lista.insert(k, mask[i])
        else:
            lista.insert(k+i, mask[i])
    return lista

#-----------------------------------------------------------------------#
#Name: get_mask_first()
#Description: removes the points that are repeated or unwanted for the 
#             first contour
#Parameters: array - array of contours
#Return: new array
#-----------------------------------------------------------------------#
def get_mask_first(array):

    mask = [True] * len(array)

    mask[0] = True

    for i in range(len(array)):
        for j in range (i+1, len(array)):
            if (distance(array[i][0], array[j][0], array[i][1], array[j][1]) < 2) & (j-i>3):
                mask[j] = False
    
    mask[-1] = True

    new_array = array[mask]

    return new_array

#-----------------------------------------------------------------------#
#Name: is_start()
#Description: Decide what is the first point for the contour in order to
#             optimize the final contour
#Parameters: array - array of contours
#Return: Boolean - True if the first point is the start point
#-----------------------------------------------------------------------#
def is_start(array):
    fst_start = array[0][0]
    fst_end = array[0][-1]

    starts = []
    ends = []
    distance_to_start = []
    distance_to_end = []

    for i in range(1,len(array)):
        #store start point and end point of all of the others contours
        starts.append(array[i][0])
        ends.append(array[i][-1])

        #bind the two arrays
        starts_ends = np.concatenate((starts, ends), axis=0)

    #calculate the distance between the start point of the first contour all the others start points in the array

    for j in range(len(starts_ends)):
        distance_to_start.append(distance(fst_start[0], starts_ends[j][0], fst_start[1], starts_ends[j][1]))
        distance_to_end.append(distance(fst_end[0], starts_ends[j][0], fst_end[1], starts_ends[j][1]))

    #calculate the average of the distances
    average_distance_to_start = np.average(distance_to_start)
    average_distance_to_end = np.average(distance_to_end)

    #choose the maximum distance
    if average_distance_to_start > average_distance_to_end:
        return True
    else:
        return False

#-----------------------------------------------------------------------#
#Name: connec_cnt()
#Description: Function that connects the contours in order to create a
#             single contour using backtracking in order to connect them
#Parameters: connected_contours - array of contours
#Return: new_contours - final array
#-----------------------------------------------------------------------#
def connec_cnt(connected_contours):
    # Copies the first contour to a new vector
    new_contours = []
    if is_start(connected_contours) == True:
        new_contours = np.copy(connected_contours[0])
    else:
        new_contours = np.copy(connected_contours[0][::-1])

    length = len(connected_contours)
    temp=[]
        
    # For each contour except the first one 
    for i in range(1,len(connected_contours)): 
        first_point=-1
        last_point=-1
        # Search if the first or last point of the next contour is close to the to a point of the original vector
        # If it is close, then it is connected to it and must be added to the new vector 
            #obtain the index of the point in the original vector
            #backtracking to the index of the point in the original vector and add all the points to the new vector
        # runs the new vector to see if the next contour is connected to any of the points in the new vector
        for j in range(new_contours.shape[0]):
            #to check if its connected or not we check the distance between the first point of the next contour and the points of the new vector 
            if (distance(connected_contours[i][0][0], new_contours[j][0], connected_contours[i][0][1], new_contours[j][1]) < 5) and (first_point==(-1)):
                #Check the index of the connection point between the two contours
                first_point=j
            #and check the distance between the last point of the next contour and the points of the new vector
            if (distance(connected_contours[i][len(connected_contours[i])-1][0], new_contours[j][0], connected_contours[i][len(connected_contours[i])-1][1], new_contours[j][1]) < 5) and (last_point==(-1)):
                last_point=j
        #After finding the point, we need to add the points from the new vector to the new vector to do the backtracking
        if(first_point>last_point):
            #this is needed to reverse the order of the points
            temp=new_contours[first_point:len(new_contours)]
            temp=temp[::-1]
            #We than try to concatenate the temp with the new vector
            new_contours=np.concatenate((new_contours, temp), axis=0)
            # add the points of the next contour to the new vector
            new_contours=np.concatenate((new_contours, connected_contours[i]), axis=0) 
        else:
            temp=new_contours[last_point:len(new_contours)]
            temp=temp[::-1]
            new_contours=np.concatenate((new_contours, temp), axis=0)
            new_contours=np.concatenate((new_contours, connected_contours[i][::-1]), axis=0)
    #Return the new vector
    return new_contours


#__________________________ROBOT_PROCESSING FUNCTIONS______________________________#

#-----------------------------------------------------------------------#
#Name: read_and_wait()
#Description: Function that reads the serial port and waits for a certain
#             amount of time
#Parameters: ser - serial port
#            wait_time - time to wait
#Return: output - string read from the serial port
#-----------------------------------------------------------------------#
def read_and_wait(ser, wait_time):
    output = ""
    flag = True
    start_time = time.time()
    time.sleep(0.5)

    while flag:
        # Wait until there is data waiting in the serial buffer
        if ser.in_waiting > 0:
            # Read data out of the buffer until a carriage return / new line is found
            serString = ser.readline()
            # Print the contents of the serial data
            try:
                output = serString.decode("Ascii")
                print(serString.decode("Ascii"))
            except:
                pass
        else:
            deltat = time.time() - start_time
            if deltat > wait_time:
                flag = False

    return output

#-----------------------------------------------------------------------#
#Name: calibrate_res()
#Description: Function that calibrates the size of the drawing based on
#             the percentage of size of an A4 paper and the size of the
#             array of points
#Parameters: square_example - array of points
#            x - x coordinate of the reference point for the drawing
#            y - y coordinate of the reference point for the drawing
#            pic_res - percentage of the size of the drawing
#Return: square_example - array of points with the new coordinates connected to
#                        the real space of the robot
#-----------------------------------------------------------------------#
def calibrate_res(square_example, x, y, pic_res):
    # A4 paper is 21.0 cm x 29.7 cm
    # 21/2 = 10.5
    # 29.7/2 = 14.85
    # Our window of operation will be a square inside the A4 paper
    # The square will be 3cm by 3cm inside the original A4 paper
    # (0,0) is bottom left corner, which is actually (3,3) cm
    _max = 0
    idx = 0
    coef = 0
    for i in range(len(square_example)):
        for j in range(2):
            # if(square_example[i][0][j] > _max):
            if (square_example[i][j] > _max):
                #_max = square_example[i][0][j]
                _max = square_example[i][j]
                idx = j
    if (idx == 0):
        coef = (pic_res * 1500) / _max  # 15 cm
    else:
        coef = (pic_res * 2370) / _max  # 23.7 cm
    for i in range(len(square_example)):
        #square_example[i][0][0] = coef * square_example[i][0][0]
        #square_example[i][0][1] = coef * square_example[i][0][1]
        square_example[i][0] = coef * square_example[i][0] + x
        square_example[i][1] = coef * square_example[i][1] + y

    return square_example
# sends the robot commands

#-----------------------------------------------------------------------#
#Name: Robot()
#Description: Function that receives the list of points, calibrates and sends
#             the commands to the robot for it to move
#Parameters: image - list of points to be drawn
#            flag  - flag to indicate if it was the first
#            g     - Reference point for the drawing
#Return: None
#-----------------------------------------------------------------------#
def Robot(image, flag, g):
    # print(image)
    print("Starting...")

    # Open the serial port COM4 to communicate with the robot (you may have to adjust
    # this instruction in case you are using a Linux OS)
    # (you must check in your computer which ports are available are, if necessary,
    # replace COM4 with the adequate COM)

    ser = serial.Serial('COM6', baudrate=9600, bytesize=8,
                        timeout=2, parity='N', xonxoff=0, stopbits=1)
    print('COM port in use: {0}'.format(ser.name))
    print("Homing the robot (if necessary)")
    ser.flush()

    # ser.write(b’home\r’)
    # time.sleep(180) # homing takes a few minutes ...

    serString = ""  # Used to hold data coming over UART

    ############################################################################
    # ATTENTION: Each point used was previously recorded with DEFP instruction
    # (from a terminal console - you can use, for example, putty or hyperterminal

    # as terminal console)
    ############################################################################

    # Creating baseline point
    # PX should be the point manually commanded to the bottom left corner of the paper
    # if(flag==0):
    print('CREATING PX...')
    ser.write(b'DEFP PX\r')
    read_and_wait(ser, 2)
    # Here we record the point and its coordinates

    print('RECORDING POSITION AS PX...')
    ser.write(b'HERE PX\r')
    read_and_wait(ser, 2)

    # Extracting points to terminal (maybe not necessary)
    f = []
    print('EXTRACTING POINTS FROM PX...')
    ser.write(b'LISTPV PX\r')
    # f = read_and_wait(ser, 2)
    time.sleep(0.5)
    print(ser.readline())
    print(ser.readline())
    print(ser.readline())
    f = ser.readline().decode('utf-8')
    # print("f is - ", f)
    g = re.findall("[XYZPR]: ?(-?\\d+)", f)
    print("Creating P1...")
    ser.write(b'SPEED 10\r')
    time.sleep(0.5)
    ser.write(b'DEFP P1\r')
    time.sleep(0.5)
    ser.write(b'SETP P1=PX\r')
    time.sleep(0.5)
    

    # Raising the Marker
    g[2] = int(g[2]) + 500

    ser.write(('SETPVC P1 z ' + str(g[2]) + '\r').encode('utf-8'))
    time.sleep(0.5)
    print(ser.readline())
    print("LIFTING THE MARKER...")
    ser.write(b'MOVE P1\r')
    time.sleep(0.5)
    print(ser.readline())
    # split the values and strings from the following format "X: int Y: int Z: int P: int R: int"

    # Setting scorbot speed to 20%

    # print('SETTING SPEED TO 5%...')
    # ser.write(b'SPEED 5\r')
    # time.sleep(0.5)
    # print(ser.readline())

    # Example vector

    #square_example = [[0, 0], [1, 0], [1, 1], [0, 1]]

    # Vector to be worked with
    #_____________________ A4_PAPER_MAPPING ___________________________# 
    n_vec = []

    # cablibrate_res() should return vector with A4 paper dimensions scaled

    n_vec = calibrate_res(image, int(g[0]), int(g[1]), 0.5)
    N = len(n_vec)
    # print("N is - ", N)
    # print("n_vec shape is - ", n_vec.shape)
    # concatenar duas strings

    # print("n_vec is - ", n_vec)
    # Vector creation
    # python compiles the following ser.write to DIMP svect[N] which the scorbot processes as the correct vector
    ser.write(('DIMP svect[' + str(N) + ']\r').encode('utf-8'))
    print(ser.readline())
    # print('DIMP' + str(vec[N-1]))
    time.sleep(0.5)

    #______________________ INVERSE_KINEMATICS ________________________#
    # Drawing algorithm:
    # Receive image coordinates, ordered in a vector (n_vec[]) by growing order in x
    # The svect[] must equal n_vec[], then run through the whole vector
    # Keep interchanging the x and the y of svect[] with n_vec[] to do so
    # Change speeds when...?
    print('CREATING VECT[N]...')
    for i in range(N):
        # Below we are initializing our svect to the only recorded position, PX
        # Below we are perfoming the operations -
        # SETPVC svect[i+1] x n_vec[i][0], which means the x coordinate of index i + 1 of svect should be the x coordinate of index i of n_vec
        # SETPVC svect[i+1] y n_vec[i][1], which means the y coordinate of index i + 1 of svect should be the y coordinate of index i of n_vec
        ser.write(('SETP svect[' + str(i + 1) + ']=PX\r').encode('utf-8'))
        print(ser.readline())
        time.sleep(0.3)
        var_1 = int(n_vec[i][0])
        var_2 = int(n_vec[i][1])
        # print("var_1 is - ", var_1, "var_2 is - ", var_2)
        ser.write(('SETPVC svect[' + str(i + 1) + '] x ' + str(var_1) + '\r').encode('utf-8'))
        time.sleep(0.3)
        print(ser.readline())
        print(ser.readline())
        print(ser.readline())
        # print("n_vec[",i,"][",1,"] is - ", n_vec[i][1])
        ser.write(('SETPVC svect[' + str(i + 1) + '] y ' + str(var_2) + '\r').encode('utf-8'))
        time.sleep(0.3)
        print(ser.readline())
        print(ser.readline())
        print(ser.readline())
        # to_stop_hundred += 1

    # In order to complete the square, the last point must be equal to the first point
    print("SETTING MARKER TO CORRECT SPOT...")
    ser.write(b'DEFP PInit\r')
    time.sleep(0.5)
    print(ser.readline())
    ser.write(('SETP PInit=svect[' + str(1) + ']\r').encode('utf-8'))
    time.sleep(0.5)
    print(ser.readline())
    ser.write(('SETPVC PInit z ' + str(g[2]) + '\r').encode('utf-8'))
    time.sleep(0.5)
    print(ser.readline())
    ser.write(b'MOVE PInit\r')
    time.sleep(0.5)
    print(ser.readline())
    ser.write(b'DEFP PF\r')
    time.sleep(0.5)
    print(ser.readline())
    ser.write(('SETP PF=svect[' + str(N) + ']\r').encode('utf-8'))
    time.sleep(0.5)
    print(ser.readline())
    print(ser.readline())
    ser.write(('SETPVC PF z ' + str(g[2]) + '\r').encode('utf-8'))
    print(ser.readline())
    time.sleep(0.5)

    speed_coef = 50 * N
    # ser.write(('GLOBAL SPD\r').encode('utf-8'))
    # time.sleep(0.2)
    # ser.write((f'SET SPD={speed_coef}\r').encode('utf-8'))
    time.sleep(0.2)
    ser.write(('MOVES svect 1 ' + str(N) + ' ' + str(speed_coef) + '\r').encode('utf-8'))
    time.sleep(0.5)
    print(ser.readline())
    print(ser.readline())
    print(ser.readline())

    # TO LIFT PEN FROM CORRECT POINT
    print("LIFTING MARKER WITHOUT DAMAGING THE DRAWING...")
    ser.write(b'MOVE PF\r')
    time.sleep(0.5)

    # ser.write(b'MOVE PT\r')
    # time.sleep(0.5)
    print("RESETTING MARKER TO ORIGINAL LIFTED POSITION...")
    ser.write(b'MOVE P1\r')
    time.sleep(0.5)
    # closing and housekeeping
    ser.close()

    print('housekeeping completed - exiting')

########################################################################

#---------------------------------------------------------------------#
#---------------------------- MAIN PROGRAM ---------------------------#
#---------------------------------------------------------------------#
def main():
    threshold = 60000 # 200 * 300
    #----------------------------------------------------------------------------#
    #                               IMPORTANT:
    # To change the image, change the name of the image in the line below
    # In order to only write the name it has to be in the same directory as this file
    #----------------------------------------------------------------------------#
    new_image = cv.imread('test_draw_2.png', cv.IMREAD_UNCHANGED)

    img_grey = cv.cvtColor(new_image,cv.COLOR_BGR2GRAY)

    thresh = 251

    ret,thresh_img = cv.threshold(img_grey, thresh, 255, cv.THRESH_BINARY)

    thresh_img = 255 - thresh_img

    #____________________ RESIZE_IMAGE ____________________#

    if ((thresh_img.shape[0] * thresh_img.shape[1]) > threshold):
        
        if (thresh_img.shape[0]<thresh_img.shape[1]):
            size = (int((thresh_img.shape[0]/thresh_img.shape[1])*100),100)
        else:
            size = (100,int((thresh_img.shape[0]/thresh_img.shape[1])*100))

        best_image = cv.resize(thresh_img,size,interpolation=cv.INTER_AREA)

    else:
        best_image = thresh_img
        
    #____________________ OBTAIN CONTOURS ____________________#
    contours, hierarchy = cv.findContours(best_image, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

    contours_list = list(contours)

    contours_list_1 = get_new_contours(contours_list)

    #remove empty contours from the list

    index_to_remove=[]

    for i in range(len(contours_list_1)):
        if contours_list_1[i].shape[0] == 0:
            index_to_remove.append(i)
            
    new_contours_list = [i for j, i in enumerate(contours_list_1) if j not in index_to_remove]

    new_list = reduce_dim(new_contours_list)

    new_points = get_mask_first(new_list[0])
    new_list[0] = new_points

    #split the contours that are poorly connected

    for i in range(len(new_list)):

        Masking = get_mask(new_list[i],4)
        
        if len(np.where(Masking)[0]) != 0:
            splited = np.split(new_list[i],np.where(Masking)[0])
            connected_list = connected(new_list,splited,i)
        else:
            connected_list=new_list
            continue

        Masking = []

    #remove contours wiht less than 3 points

    index_to_remove=[]
    for i in range(len(connected_list)):
        if len(connected_list[i]) < 4:
            index_to_remove.append(i)

    my_new_best_list = [i for j, i in enumerate(connected_list) if j not in index_to_remove]

    #____________________ PRINTING_VECTOR ____________________#
    vector = connec_cnt(my_new_best_list)
    column = vector[:, 1]
    inverted_column = -column
    vector[:, 1] = inverted_column + size[1]

    #----------------------------------------------------------------------------#
    #                               IMPORTANT:
    # To diminish the number of points in the vector change the value in the mask
    # The higher the value the less points in the vector (distance between points)
    #----------------------------------------------------------------------------#
    vector = vector[get_mask2(vector, 6)]
    # square vector
    vector_2 = [[0.1,0.1], [0,1], [0.5,0.5], [0.5,0],[0.1,0.1]]

    g=[0.0,0.0,0.0,0.0,0.0]
    Robot(vector, 0, g)

if __name__ == "__main__":
    main()
