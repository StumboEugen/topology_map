//
// Created by stumbo on 18-5-16.
//

#include "TopoEdge.h"
#include "MapCandidate.h"
#include "TopoNode.h"

/**
 * change one exit to another exit
 * @attention the node related will be modified too
 * @param oldNode
 * @param newNode
 * @param newGate
 */
void TopoEdge::changeExitTo(TopoNode *const oldNode, TopoNode *const newNode, const unsigned char newGate) {
    if (oldNode == exitB) {
        exitB->disconnectEdge(gateB);
        exitB = newNode;
        gateB = newGate;
        exitB->addEdge(gateB, this);
    } else if (oldNode == exitA) {
        exitA->disconnectEdge(gateA);
        exitA = newNode;
        gateA = newGate;
        exitA->addEdge(gateA, this);
    } else {
        cout << "TopoEdge changeExitTo FAILURE" << endl;
        throw;
    }
}