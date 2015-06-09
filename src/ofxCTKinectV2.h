#ifndef CT_KINECT_V2
#define CT_KINECT_V2

#include "ofMain.h"
#include <Kinect.h>

static const int cDEPTH_WIDTH = 512;
static const int cDEPTH_HEIGHT = 424;
static const int cCOLOR_WIDTH = 1920;
static const int cCOLOR_HEIGHT = 1080;
static const int cCOLOR_CHANNEL = 4;

typedef struct _stSCREEN_SKELETON
{
	UINT64		ui64TrackingID;
	bool		bIsTracking;
	ofVec2f		aJoints[JointType_Count];
	void clear()
	{
		ui64TrackingID = 0;
		bIsTracking = false;
		for(auto Joint:aJoints)
		{
			Joint.set(0);
		}
	}
	HandState	eLeftHandState, eRightHandState;

}stSCREEN_SKELETON;

class CTKinectV2 : public ofThread
{
public:
	CTKinectV2()
		:_bIsSetup(false)
		,_bEnableBodyIndex(false)
		,_bEnableSkeleton(false)
		,_bEnableColorFrame(false)
		,_bIsNewBodyIndex(false)
		,_bIsNewColorFrame(false)
		,_bHaveUser(false)
		,_BodyIndex(255)
		,_pKinectSensor(nullptr)
		,_pCoordinateMapper(nullptr)
		,_pBodyIndexFrameReader(nullptr)
		,_pBodyFrameReader(nullptr)
		,_pColorFrameReader(nullptr)
	{}
	~CTKinectV2();

////////////////////////////////
//public member method
////////////////////////////////
public:
	bool initialKinectV2();
	void updateKinectV2();

	bool getBodyIndex(ofImage& refImg);
	bool getColorFrame(ofImage& refImg);
	bool getJoint(int iJointID, ofVec2f& JointPos) const;
	bool getSkeleton(stSCREEN_SKELETON& Skeleton) const;

	void enableBodyIndex();
	void disableBodyIndex();

	void enableSkeleton();
	void disableSkeleton();

	void enableColorFrame();
	void disableColorFrame();


////////////////////////////////
//private member method
////////////////////////////////
private:
	void updateBodyIndex();
	void updateSkeleton();
	void updateColorFrame();

////////////////////////////////
//private inline member method
////////////////////////////////
private:
	inline bool checkFunc(HRESULT hr, string strFunc, int iLine)
	{
		if(SUCCEEDED(hr))
		{
			return true;
		}
		else if(hr == E_FAIL)
		{
			ofLog(OF_LOG_ERROR, "Kinect error!! in " + strFunc + " line:" + ofToString(iLine));
			return false;
		}
		return false;
	}

	template<typename T>
	void releaseKinectObj(T ptr)
	{
		if(ptr != nullptr)
		{
			ptr->Release();
		}
	}

private:
	IKinectSensor*			_pKinectSensor;
	ICoordinateMapper*		_pCoordinateMapper;

	IBodyIndexFrameReader*	_pBodyIndexFrameReader;
	IBodyFrameReader*		_pBodyFrameReader;
	IColorFrameReader*		_pColorFrameReader;

	bool					_bIsSetup;
	bool					_bEnableBodyIndex, _bEnableSkeleton, _bEnableColorFrame;
	bool					_bIsNewBodyIndex, _bIsNewColorFrame;
	bool					_bHaveUser;

	ofPixels				_BodyIndexPixels, _ColorFramePixels;
	stSCREEN_SKELETON		_Skeleton;
	int						_BodyIndex;
//------------------------------
//Thread main loop
//------------------------------
public:
	void threadedFunction()
	{
		while( isThreadRunning() != 0)
		{	
			if(!_bIsSetup)
			{
				continue;
			}

			if( lock() )
			{
				this->updateKinectV2();
			}
			unlock();
		}
	};
};
#endif // !CT_KINECT_V2
