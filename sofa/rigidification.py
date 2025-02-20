
# utility methods for rigidification

# We suppose that we have a mesh with a given number of nodes
# - a subpart of these nodes are 'free'
# - the remaining part of these nodes are grouped in 'rigidified' blocks

# for the mapping, we need to build a list of index pairs, 1 pair / nodes
# [ [model Origin (free = 0, rigidified = 1 to n)   number in the subpart list]

# see the exemple described in the test for a better understanding...


import numpy

def fillIndexPairs(numNodes, freeNodes, rigidBlocks):
    numRigidBlocks = len(rigidBlocks)


    freeSortTab = numpy.argsort(freeNodes)
    freeNodes.sort()
    rigidSortTab = numpy.argsort(rigidBlocks[0])
    rigidBlocks[0].sort()
    # TODO verify that the lists of freeNodes and rigidBlocks are provided in ascending order !
    iteratorFreeNodes=0;
    iteratorRigidBlocks=[0]*numRigidBlocks

    #  iterators limits
    numFreeNodes = len(freeNodes);
    numRigidNodesOnBlock = [0]*numRigidBlocks


    for j in range (numRigidBlocks):
        numRigidNodesOnBlock[j] =  len(rigidBlocks[j])

    # test num Nodes:
    numNodes2 = numFreeNodes+sum(numRigidNodesOnBlock)
    #print 'numNodes2 :' +str(numNodes2)

    if numNodes != numNodes2:
        print '+++++++++++++++++++++++++++++++++++++++++++++'
        print 'error in the lists freeNodes and rigidBlocks'
        print '+++++++++++++++++++++++++++++++++++++++++++++'
        print 'numNodes = '+ str(numNodes)+ ' does not corresponds to total size of freeNodes + rigidBlocks'
        print 'freeNodes: size ='+ str(numFreeNodes)
        print 'rigidBlocks: size = '+ str(sum(numRigidNodesOnBlock))
        print '+++++++++++++++++++++++++++++++++++++++++++++'
        return [ ]


    #result:
    indexPairs=[1,1]*(numNodes)

    for i in range(numNodes):

        if i==freeNodes[iteratorFreeNodes]:
            # node i belongs to free Nodes
            indexPairs[2*i  ]=0;
            indexPairs[2*i+1]=freeSortTab[iteratorFreeNodes];
            iteratorFreeNodes=iteratorFreeNodes+1;

            if (iteratorFreeNodes>=numFreeNodes): # we are at the limit
                iteratorFreeNodes = iteratorFreeNodes-1;


        else:
            for j in range (numRigidBlocks):
                if i==rigidBlocks[j][iteratorRigidBlocks[j]]:
                    indexPairs[2*i  ]=j+1;
                    indexPairs[2*i+1]=rigidSortTab[iteratorRigidBlocks[j]];
                    iteratorRigidBlocks[j] = iteratorRigidBlocks[j]+1;

                    if (iteratorRigidBlocks[j]>=numRigidNodesOnBlock[j]): # we are at the limit of the node list
                        iteratorRigidBlocks[j] = iteratorRigidBlocks[j]-1;


    return indexPairs



def fillIndexPairs_SeveralFree(numNodes, freeBlocks, rigidBlocks):
    numRigidBlocks = len(rigidBlocks)
    nbFreeBlocks = len(freeBlocks)
    print 'freeBlocks', freeBlocks
    print 'rigidBlocks', rigidBlocks

    # TODO verify that the lists of freeNodes and rigidBlocks are provided in ascending order !
    iteratorFreeBlocks=[0]*nbFreeBlocks;
    iteratorRigidBlocks=[0]*numRigidBlocks


    #  iterators limits
    numFreeNodesOnBlock = [0]*nbFreeBlocks
    numRigidNodesOnBlock = [0]*numRigidBlocks


    for j in range (numRigidBlocks):
        numRigidNodesOnBlock[j] =  len(rigidBlocks[j])
    for j in range (nbFreeBlocks):
        numFreeNodesOnBlock[j] =  len(freeBlocks[j])
    # test num Nodes:
    numNodes2 = sum(numFreeNodesOnBlock) + sum(numRigidNodesOnBlock)
    #print 'numNodes2 :' +str(numNodes2)

    if numNodes != numNodes2:
        print '+++++++++++++++++++++++++++++++++++++++++++++'
        print 'error in the lists freeNodes and rigidBlocks'
        print '+++++++++++++++++++++++++++++++++++++++++++++'
        print 'numNodes = '+ str(numNodes)+ ' does not corresponds to total size of freeNodes + rigidBlocks'
        print 'freeNodes: size ='+ str(sum(numFreeNodesOnBlock))
        print 'rigidBlocks: size = '+ str(sum(numRigidNodesOnBlock))
        print '+++++++++++++++++++++++++++++++++++++++++++++'
        return [ ]


    #result:
    indexPairs=[1,1]*(numNodes)
    print 'nbFreeBlocks ', nbFreeBlocks
    print 'numRigidBlocks', numRigidBlocks
    for i in range(numNodes):
        #print 'i is :', i
        for j in range (nbFreeBlocks):
            #print 'num free block:', j
            #print 'freeBlocks[j][iteratorFreeBlocks[j]] ', freeBlocks[j][iteratorFreeBlocks[j]]
            if i==freeBlocks[j][iteratorFreeBlocks[j]]:
                # node i belongs to free Nodes
                #print 'In free Nodes, setting [', j,', ', iteratorFreeBlocks[j],']'
                indexPairs[2*i  ]=j;
                indexPairs[2*i+1]=iteratorFreeBlocks[j];
                iteratorFreeBlocks[j] = iteratorFreeBlocks[j]+1;

                if (iteratorFreeBlocks[j]>=numFreeNodesOnBlock[j]): # we are at the limit
                    iteratorFreeBlocks[j] = iteratorFreeBlocks[j]-1;


        for j in range (numRigidBlocks):
            #print 'num rigid block:', j
            #print 'rigidBlocks[j][iteratorRigidBlocks[j]] ', rigidBlocks[j][iteratorRigidBlocks[j]]
            if i==rigidBlocks[j][iteratorRigidBlocks[j]]:
                    #print 'In rigid Nodes, setting [', j+ nbFreeBlocks,', ', iteratorRigidBlocks[j],']'
                    indexPairs[2*i  ]=j + nbFreeBlocks;
                    indexPairs[2*i+1]=iteratorRigidBlocks[j];
                    iteratorRigidBlocks[j] = iteratorRigidBlocks[j]+1;

                    if (iteratorRigidBlocks[j]>=numRigidNodesOnBlock[j]): # we are at the limit of the node list
                        iteratorRigidBlocks[j] = iteratorRigidBlocks[j]-1;


    return indexPairs

def fillIndexPairs2(numNodes, rigidBlocks):
    numRigidBlocks = len(rigidBlocks)

    # TODO verify that the lists of freeNodes and rigidBlocks are provided in ascending order !
    iteratorFreeNodes=0;
    iteratorRigidBlocks=[0]*numRigidBlocks
    numRigidNodesOnBlock = [0]*numRigidBlocks


    for j in range (numRigidBlocks):
        numRigidNodesOnBlock[j] =  len(rigidBlocks[j])
        #print 'numRigidNodesOnBlock['+str(j)+'] = ' + str(numRigidNodesOnBlock[j])
        #print rigidBlocks[j]




    #result:
    indexPairs=[1,1]*(numNodes)

    for i in range(numNodes):

        freeNodeTest=1;
        for j in range (numRigidBlocks):
            if i==rigidBlocks[j][iteratorRigidBlocks[j]]:
                freeNodeTest=0;
                indexPairs[2*i  ]=j+1;
                indexPairs[2*i+1]=iteratorRigidBlocks[j];
                iteratorRigidBlocks[j] = iteratorRigidBlocks[j]+1;

                if (iteratorRigidBlocks[j]>=numRigidNodesOnBlock[j]): # we are at the limit of the node list
                    iteratorRigidBlocks[j] = iteratorRigidBlocks[j]-1;

        if freeNodeTest:
            # node i belongs to free Nodes
            indexPairs[2*i  ]=0;
            indexPairs[2*i+1]=iteratorFreeNodes;
            iteratorFreeNodes=iteratorFreeNodes+1;


    return indexPairs



def fillIndexPairs_oneRigid(numNodes, rigidBlocks):
    numRigidBlocks = len(rigidBlocks)
    indexPairs=[1,1]*(numNodes)

    iteratorFreeNodes=0;
    iteratorRigidNodes=0;

    for i in range(numNodes):
        freeNodeTest=1;
        iteratorRigidNodes = 0;

        it=0;

        for j in range (numRigidBlocks):

            for k in rigidBlocks[j]:
                if i==k:
                    freeNodeTest=0;
                    iteratorRigidNodes=it
                it=it+1


        if freeNodeTest:
            # node i belongs to free Nodes
            indexPairs[2*i  ]=0;
            indexPairs[2*i+1]=iteratorFreeNodes;
            iteratorFreeNodes=iteratorFreeNodes+1;
        else:
            indexPairs[2*i  ]=1;
            indexPairs[2*i+1]=iteratorRigidNodes;


    return indexPairs


# compute the gravity center of a set of point (list of position)

def gravityCenter(listPos):
    numP = len(listPos)
    posG=[0,0,0]
    for p in range(numP):
        posG = [ posG[0]+ listPos[p][0],  posG[1]+ listPos[p][1],  posG[2]+ listPos[p][2] ]  ;

    posG = [posG[0]/numP, posG[1]/numP, posG[2]/numP ];

    return posG







def doTest():
    # example TEST
    # lets consider that we have a mesh with 10 nodes
    # free: nodes 0 2 3 4 6
    # 1st rigid block: nodes 1 7 9
    # 2d rigid block: nodes 5 8

    # the function should provide:
    # free nodes: [0 0] [0 1] [0 2] [0 3] [0 4]
    # 1st rigid block: nodes [1 0] [1 1] [1 2]
    # 2d rigid block: nodes [2 0] [2 1]

    # the method should provide the following pairs
    # [[0 0] [1 0] [0 1] [0 2] [0 3] [2 0] [0 4] [1 1] [2 1] [1 2]]
    # and the mapping of SOFA requires a simple list:
    # [0,0,1,0,0,1,0,2,0,3,2,0,0,4,1,1,2,1,1,2]

    freeNodes=[0, 2, 3, 4, 6];
    rigidBlocks=[[1,7,9], [5,8]];
    numNodes=10;


    index_result = fillIndexPairs(numNodes, freeNodes, rigidBlocks);
    print 'result='
    print index_result

    index_result = fillIndexPairs2(numNodes, rigidBlocks);
    print 'result='
    print index_result


    if [0,0,1,0,0,1,0,2,0,3,2,0,0,4,1,1,2,1,1,2] == index_result:
        print ('result ok')
    else:
        print ('error')

    return



if __name__ == '__main__':
    doTest()

