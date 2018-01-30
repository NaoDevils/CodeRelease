/**
 * @file MocapDataProvider.cpp
 * This file implements a module that provides data from the motion capturing system
 * @author Janine Hemmers
 */

#include "MocapDataProvider.h"

PROCESS_LOCAL MocapDataProvider* MocapDataProvider::theInstance = 0;

MocapDataProvider::MocapDataProvider() 
{
  theInstance = this;
}

MocapDataProvider::~MocapDataProvider()
{
  theInstance = 0;
}


void MocapDataProvider::update(MocapData& data)
{
  mocapDataPtr = &data;
  //data.rigidBodies.clear();
}

void MocapDataProvider::handleMessages(MocapDataIn& mocapReceiver)
{
  std::vector<MocapMessage> cpy_messages(mocapReceiver.messages);
  mocapReceiver.clear();
  //OUTPUT_TEXT(cpy_messages.size());
  /* Unpack the newest MocapMessage */
  
  if (theInstance) 
  {  
    int counter = 0;
    int currFrameNumber = 0;
    for (unsigned i = 0; i < cpy_messages.size(); i++) 
    {
      //OUTPUT_TEXT("Get Framenumber");
      if (currFrameNumber < theInstance->getFramenumber((char*)&cpy_messages[i])) 
      {
        currFrameNumber = theInstance->getFramenumber((char*)&cpy_messages[i]);
        counter = i;
      }
    } 
    
    if (cpy_messages.size() > 0)
    {
      //OUTPUT_TEXT("unpack");
      theInstance->unpack((char*)&cpy_messages[counter]);
    } 
  }
 
}

int MocapDataProvider::getFramenumber(char * pData)
{
  char *ptr = pData;

  // message ID
  int MessageID = 0;
  memcpy(&MessageID, ptr, 2);
  ptr += 4;

  if (MessageID == NAT_FRAMEOFDATA)
  {
    // frame number
    int frameNumber = 0; memcpy(&frameNumber, ptr, 4);
    return frameNumber;
  }
  return 0;
}

void MocapDataProvider::unpack(char* pData)
{
    int major = 2;
    int minor = 10;

    char *ptr = pData;

    // message ID
    int MessageID = 0;
    memcpy(&MessageID, ptr, 2);
    ptr += 2;
   
    if (MessageID == NAT_FRAMEOFDATA)
    {

      // size
      //int nBytes = 0;
      //memcpy(&nBytes, ptr, 2); 
      ptr += 2;

      // frame number
      int frameNumber = 0; memcpy(&frameNumber, ptr, 4);
      ptr += 4;
      mocapDataPtr->frameNumber = frameNumber;

      // number of data sets (markersets, rigidbodies, etc)
      int nMarkerSets = 0; memcpy(&nMarkerSets, ptr, 4);
      ptr += 4;

      for (int i = 0; i < nMarkerSets; i++)
      {
        // Markerset name
        char szName[256];
        strcpy(szName, ptr);
        MocapMarkerSet mms(szName);
        int nDataBytes = (int)strlen(szName) + 1;
        ptr += nDataBytes;

        // marker data
        int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;

        for (int j = 0; j < nMarkers; j++)
        {
          float x = 0; memcpy(&x, ptr, 4); 
          ptr += 4;
          float y = 0; memcpy(&y, ptr, 4); 
          ptr += 4;
          float z = 0; memcpy(&z, ptr, 4); 
          ptr += 4;
          mms.marker.push_back(Vector3f(x, y, z));
          //printf("\tMarker %d : [x=%3.2f,y=%3.2f,z=%3.2f]\n", j, x, y, z);
        }
        if(mms.name.compare(settings.robotName)==0)
        {
          mocapDataPtr->markerSet=mms;
          //OUTPUT_TEXT(mms.name);
        }
        if (mms.name.compare("Ball") == 0)
        {
          mocapDataPtr->markerSetBall = mms;
        }
      }

      // unidentified markers
      int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
      //printf("Unidentified Marker Count : %d\n", nOtherMarkers);
      for (int j = 0; j < nOtherMarkers; j++)
      {
        //float x = 0.0f; memcpy(&x, ptr, 4); 
        ptr += 4;
        //float y = 0.0f; memcpy(&y, ptr, 4); 
        ptr += 4;
        //float z = 0.0f; memcpy(&z, ptr, 4); 
        ptr += 4;
        //printf("\tMarker %d : pos = [%3.2f,%3.2f,%3.2f]\n", j, x, y, z);
      }

      // rigid bodies
      int nRigidBodies = 0;
      memcpy(&nRigidBodies, ptr, 4); ptr += 4;
      //printf("Rigid Body Count : %d\n", nRigidBodies);
      mocapDataPtr->rigidBodies.clear();
      for (int j = 0; j < nRigidBodies; j++)
      {
        // rigid body pos/ori
        int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
        float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
        float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
        float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
        float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
        float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
        float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
        float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
        //printf("ID : %d\n", ID);
        //printf("pos: [%3.2f,%3.2f,%3.2f]\n", x, y, z);
        //printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx, qy, qz, qw);
        MocapRigidbody rig(ID, x, y, z, qx, qy, qz, qw);

        // associated marker positions
        int nRigidMarkers = 0;  memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
        rig.numberOfMarkers = nRigidMarkers;
        //printf("Marker Count: %d\n", nRigidMarkers);
        int nBytes = nRigidMarkers * 3 * sizeof(float);
        float* markerData = (float*)malloc(nBytes);
        memcpy(markerData, ptr, nBytes);
        rig.firstMarker = Vector3f(markerData[0], markerData[1], markerData[2]);
        ptr += nBytes;


        if (major >= 2)
        {
          // associated marker IDs
          nBytes = nRigidMarkers * sizeof(int);
          //int* markerIDs = (int*)malloc(nBytes);
          //memcpy(markerIDs, ptr, nBytes);
          //rig.markerIDs = markerIDs;
          ptr += nBytes;

          // associated marker sizes
          nBytes = nRigidMarkers * sizeof(float);
          //float* markerSizes = (float*)malloc(nBytes);
          //memcpy(markerSizes, ptr, nBytes);
          //rig.markerSizes = markerSizes;
          ptr += nBytes;

          
          /*
          for (int k = 0; k < nRigidMarkers; k++)
          {
          //printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k], markerData[k * 3], markerData[k * 3 + 1], markerData[k * 3 + 2]);
          }
          */
        
          //if (markerIDs)
          //  free(markerIDs);
          //if (markerSizes)
          //  free(markerSizes);

        }
        else
        {
          for (int k = 0; k < nRigidMarkers; k++)
          {
            //printf("\tMarker %d: pos = [%3.2f,%3.2f,%3.2f]\n", k, markerData[k * 3], markerData[k * 3 + 1], markerData[k * 3 + 2]);
          }
        }
        if (markerData)
          free(markerData);

        if (major >= 2)
        {
          // Mean marker error
          float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
          //printf("Mean marker error: %3.2f\n", fError);
          rig.meanError = fError;
        }

        // 2.6 and later
        if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0))
        {
          // params
          short params = 0; memcpy(&params, ptr, 2); ptr += 2;
          bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
          rig.trackingValid = bTrackingValid;
        }

        //if (mocapDataPtr->rigidBodies.size() == 0) {
          mocapDataPtr->rigidBodies.push_back(rig);
        //}

      } // next rigid body


        // skeletons (version 2.1 and later)
      if (((major == 2) && (minor > 0)) || (major > 2))
      {
        int nSkeletons = 0;
        memcpy(&nSkeletons, ptr, 4); ptr += 4;
        //printf("Skeleton Count : %d\n", nSkeletons);
        for (int j = 0; j < nSkeletons; j++)
        {
          // skeleton id
          //int skeletonID = 0;
          //memcpy(&skeletonID, ptr, 4); 
          ptr += 4;
          // # of rigid bodies (bones) in skeleton
          int nRigidBodies = 0;
          memcpy(&nRigidBodies, ptr, 4); ptr += 4;
          //printf("Rigid Body Count : %d\n", nRigidBodies);
          for (int j = 0; j < nRigidBodies; j++)
          {
            // rigid body pos/ori
            /*
            int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
            float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
            float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
            float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
            float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
            float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
            float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
            float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
            */


            ptr += 32;
            //printf("ID : %d\n", ID);
            //printf("pos: [%3.2f,%3.2f,%3.2f]\n", x, y, z);
            //printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx, qy, qz, qw);

            // associated marker positions
            int nRigidMarkers = 0;  memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
            //printf("Marker Count: %d\n", nRigidMarkers);
            int nBytes = nRigidMarkers * 3 * sizeof(float);
            float* markerData = (float*)malloc(nBytes);
            memcpy(markerData, ptr, nBytes);
            ptr += nBytes;

            // associated marker IDs
            nBytes = nRigidMarkers * sizeof(int);
            int* markerIDs = (int*)malloc(nBytes);
            memcpy(markerIDs, ptr, nBytes);
            ptr += nBytes;

            // associated marker sizes
            nBytes = nRigidMarkers * sizeof(float);
            float* markerSizes = (float*)malloc(nBytes);
            memcpy(markerSizes, ptr, nBytes);
            ptr += nBytes;

            /*
            for (int k = 0; k < nRigidMarkers; k++)
            {
            //printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k], markerData[k * 3], markerData[k * 3 + 1], markerData[k * 3 + 2]);
            }*/

            // Mean marker error (2.0 and later)
            if (major >= 2)
            {
              //float fError = 0.0f; memcpy(&fError, ptr, 4); 
              ptr += 4;
              //printf("Mean marker error: %3.2f\n", fError);
            }

            // Tracking flags (2.6 and later)
            if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0))
            {
              // params
              //short params = 0; memcpy(&params, ptr, 2); 
              ptr += 2;
              //bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
            }

            // release resources
            if (markerIDs)
              free(markerIDs);
            if (markerSizes)
              free(markerSizes);
            if (markerData)
              free(markerData);

          } // next rigid body

        } // next skeleton
      }

      // labeled markers (version 2.3 and later)
      if (((major == 2) && (minor >= 3)) || (major > 2))
      {
        int nLabeledMarkers = 0;
        memcpy(&nLabeledMarkers, ptr, 4); ptr += 4;
        //printf("Labeled Marker Count : %d\n", nLabeledMarkers);
        for (int j = 0; j < nLabeledMarkers; j++)
        {
          /*
          // id
          int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
          // x
          float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
          // y
          float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
          // z
          float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
          // size
          float size = 0.0f; memcpy(&size, ptr, 4); ptr += 4;
          */

          ptr += 20;

          // 2.6 and later
          if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0))
          {
            // marker params
            //short params = 0; memcpy(&params, ptr, 2); 
            ptr += 2;
            //bool bOccluded = params & 0x01;     // marker was not visible (occluded) in this frame
            //bool bPCSolved = params & 0x02;     // position provided by point cloud solve
            //bool bModelSolved = params & 0x04;  // position provided by model solve
          }

          //printf("ID  : %d\n", ID);
          //printf("pos : [%3.2f,%3.2f,%3.2f]\n", x, y, z);
          //printf("size: [%3.2f]\n", size);
        }
      }

      // Force Plate data (version 2.9 and later)
      if (((major == 2) && (minor >= 9)) || (major > 2))
      {
        int nForcePlates;
        memcpy(&nForcePlates, ptr, 4); ptr += 4;
        for (int iForcePlate = 0; iForcePlate < nForcePlates; iForcePlate++)
        {
          // ID
          //int ID = 0; memcpy(&ID, ptr, 4); 
          ptr += 4;
          //printf("Force Plate : %d\n", ID);

          // Channel Count
          int nChannels = 0; memcpy(&nChannels, ptr, 4); ptr += 4;

          // Channel Data
          for (int i = 0; i < nChannels; i++)
          {
            //printf(" Channel %d : ", i);
            int nFrames = 0; memcpy(&nFrames, ptr, 4); ptr += 4;
            for (int j = 0; j < nFrames; j++)
            {
              //float val = 0.0f;  memcpy(&val, ptr, 4); 
              ptr += 4;
              //printf("%3.2f   ", val);
            }
            //printf("\n");
          }
        }
      }

      // latency
      float latency = 0.0f; memcpy(&latency, ptr, 4);	ptr += 4;
      //printf("latency : %3.3f\n", latency);

      // timecode
      unsigned int timecode = 0; 	memcpy(&timecode, ptr, 4);	ptr += 4;
      unsigned int timecodeSub = 0; memcpy(&timecodeSub, ptr, 4); ptr += 4;
      //char szTimecode[128] = "";
      //TimecodeStringify(timecode, timecodeSub, szTimecode, 128);
      //int hour, minute, second, frame, subframe;
      //DecodeTimecode(timecode, timecodeSub, &hour, &minute, &second, &frame, &subframe);
      //OUTPUT_TEXT("h:" << hour << " m:" << minute << " s:" << second << " f:" << frame << " s" << subframe);


      // timestamp
      double timestamp = 0.0f;
      // 2.7 and later - increased from single to double precision
      if (((major == 2) && (minor >= 7)) || (major > 2))
      {
        memcpy(&timestamp, ptr, 8); ptr += 8;
      }
      else
      {
        float fTemp = 0.0f;
        memcpy(&fTemp, ptr, 4); ptr += 4;
        timestamp = (double)fTemp;
      }


      // frame params
      //short params = 0;  memcpy(&params, ptr, 2); ptr += 2;
      //bool bIsRecording = params & 0x01;                  // 0x01 Motive is recording
      //bool bTrackedModelsChanged = params & 0x02;         // 0x02 Actively tracked model list has changed


      // end of data tag
      //int eod = 0; memcpy(&eod, ptr, 4); ptr += 4;
      //printf("End Packet\n-------------\n");
    }
}

MAKE_MODULE(MocapDataProvider, cognitionInfrastructure)