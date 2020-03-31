class Joint{
 private PVector [] storedPositions;
 private int positionIndex, currentPosition;
 private int hopSize = 3;
 //private PVector posTplus2;
 //private PVector posTplus1;
 //private PVector posT;
 //private PVector posTminus1;
 //private PVector posTminus2;
 
 public Joint(){
   positionIndex = 0;
   currentPosition = 0;
   storedPositions = new PVector [hopSize*5];
   for(int i = 0; i<storedPositions.length; i++){
     storedPositions[i] = new PVector(0,0);
   }
   //posTplus2 = new PVector(0,0);
   //posTplus1 = new PVector(0,0);
   //posT = new PVector(0,0);
   //posTminus1 = new PVector(0,0);
   //posTminus2 = new PVector(0,0);
 }
 
 void setNewPosition(int x, int y){
   storedPositions[positionIndex].set(x,y); //<>//
   positionIndex++; //<>//
   currentPosition = positionIndex-1; //<>//
   positionIndex = positionIndex%storedPositions.length; //<>//
    //<>//
   //println("set a new Position to: " + storedPositions[positionIndex].x);
   //posTminus2 = new PVector(posTminus1.x, posTminus1.y);
   //posTminus1 = new PVector(posT.x, posT.y);
   //posT = new PVector(posTplus1.x, posTplus1.y);
   //posTplus1 = new PVector(posTplus2.x, posTplus2.y);
   //posTplus2 = posTplus2.set(x,y);
 }
 PVector getPosPlus2(){
   return storedPositions[currentPosition];
 }
 PVector getPosPlus1(){
   return storedPositions[(currentPosition+hopSize*4)%storedPositions.length];
 }
 PVector getPos(){
   return storedPositions[(currentPosition+hopSize*3)%storedPositions.length];
 }
  PVector getPosMinus1(){
   return storedPositions[(currentPosition+hopSize*2)%storedPositions.length];
 }
 PVector getPosMinus2(){
   return storedPositions[(currentPosition+hopSize)%storedPositions.length];
 }
}