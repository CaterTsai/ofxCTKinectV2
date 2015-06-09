#include "ofxCTKinectV2.h"

CTKinectV2::~CTKinectV2()
{
	this->releaseKinectObj(_pKinectSensor);
	this->releaseKinectObj(_pCoordinateMapper);
	this->releaseKinectObj(_pBodyIndexFrameReader);
	this->releaseKinectObj(_pBodyFrameReader);
	this->releaseKinectObj(_pColorFrameReader);
}

//--------------------------------------------------------------
bool CTKinectV2::initialKinectV2()
{
	if( !this->checkFunc( GetDefaultKinectSensor(&_pKinectSensor), __FILE__, __LINE__) )
	{
		return false;
	}

	if(_pKinectSensor)
	{
		if( !this->checkFunc( _pKinectSensor->Open(), __FILE__, __LINE__) )
		{
			return false;	
		}
		
		//Coordinate
		if( !this->checkFunc( _pKinectSensor->get_CoordinateMapper(&_pCoordinateMapper), __FILE__, __LINE__) )
		{
			return false;
		}

		//Body index initial
		IBodyIndexFrameSource* pBodyIndexFrameSource_ = nullptr;
		if( !checkFunc( _pKinectSensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource_), __FILE__, __LINE__ ))
		{
			this->releaseKinectObj(pBodyIndexFrameSource_);
			return false;
		}
		if( !checkFunc( pBodyIndexFrameSource_->OpenReader(&_pBodyIndexFrameReader), __FILE__, __LINE__))
		{
			this->releaseKinectObj(pBodyIndexFrameSource_);
			return false;
		}
		this->releaseKinectObj(pBodyIndexFrameSource_);
		_BodyIndexPixels.allocate(cDEPTH_WIDTH, cDEPTH_HEIGHT, ofImageType::OF_IMAGE_COLOR);

		//Body initial
		IBodyFrameSource* pBodyFrameSource_ = nullptr;
		if( !checkFunc( _pKinectSensor->get_BodyFrameSource(&pBodyFrameSource_), __FILE__, __LINE__ ))
		{
			this->releaseKinectObj(pBodyFrameSource_);
			return false;
		}
		if( !checkFunc( pBodyFrameSource_->OpenReader(&_pBodyFrameReader), __FILE__, __LINE__))
		{
			this->releaseKinectObj(pBodyFrameSource_);
			return false;
		}
		this->releaseKinectObj(pBodyFrameSource_);

		//Color frame initial
		IColorFrameSource* pColorFrameSource_ = nullptr;
		if( !checkFunc( _pKinectSensor->get_ColorFrameSource(&pColorFrameSource_), __FILE__, __LINE__) )
		{
			this->releaseKinectObj(pColorFrameSource_);
			return false;
		}
		if( !checkFunc( pColorFrameSource_->OpenReader(&_pColorFrameReader), __FILE__, __LINE__) )
		{
			this->releaseKinectObj(pColorFrameSource_);
			return false;
		}
		this->releaseKinectObj(pColorFrameSource_);
		_ColorFramePixels.allocate(cCOLOR_WIDTH, cCOLOR_HEIGHT, ofImageType::OF_IMAGE_COLOR);
	}

	_bIsSetup = true;
	return true;
}

//--------------------------------------------------------------
void CTKinectV2::updateKinectV2()
{
	if(_bEnableBodyIndex)
	{
		this->updateBodyIndex();
	}
	
	if(_bEnableSkeleton)
	{
		this->updateSkeleton();
	}

	if(_bEnableColorFrame)
	{
		this->updateColorFrame();
	}
}

//--------------------------------------------------------------
bool CTKinectV2::getBodyIndex(ofImage& refImg)
{
	if(!_bIsSetup || !_bEnableBodyIndex || !_bIsNewBodyIndex)
	{
		return false;
	}
	
	lock();
	refImg.setFromPixels(_BodyIndexPixels.getPixels(), cDEPTH_WIDTH, cDEPTH_HEIGHT, ofImageType::OF_IMAGE_COLOR);
	unlock();

	return true;
}

//--------------------------------------------------------------
bool CTKinectV2::getColorFrame(ofImage& refImg)
{
	if(!_bIsSetup || !_bEnableColorFrame || !_bIsNewColorFrame)
	{
		return false;
	}

	_bIsNewColorFrame = false;
	lock();
	refImg.setFromPixels(_ColorFramePixels.getPixels(), cCOLOR_WIDTH, cCOLOR_HEIGHT, ofImageType::OF_IMAGE_COLOR);
	unlock();

	return true;
}

//--------------------------------------------------------------
bool CTKinectV2::getJoint(int iJointID, ofVec2f& refJointPos) const
{
	refJointPos.set(0);

	if(!_bIsSetup || !_bEnableSkeleton || !_bHaveUser)
	{
		return false;
	}

	if(iJointID < 0 || iJointID >= JointType_Count)
	{
		ofLog(OF_LOG_ERROR, "Wrong joint ID : " + ofToString(iJointID));
		return false;
	}
	else
	{
		refJointPos = _Skeleton.aJoints[iJointID];
	}

	return _bHaveUser;
}

//--------------------------------------------------------------
bool CTKinectV2::getSkeleton(stSCREEN_SKELETON& refSkeleton) const
{
	refSkeleton.clear();
	if(!_bIsSetup || !_bEnableSkeleton || !_bHaveUser)
	{
		return false;
	}
	
	refSkeleton = _Skeleton;
	return _bHaveUser;
}

//--------------------------------------------------------------
void CTKinectV2::enableBodyIndex()
{
	_bEnableBodyIndex = true;
}

//--------------------------------------------------------------
void CTKinectV2::disableBodyIndex()
{
	_bEnableBodyIndex = false;
}

//--------------------------------------------------------------
void CTKinectV2::enableSkeleton()
{
	_bEnableSkeleton = true;
}

//--------------------------------------------------------------
void CTKinectV2::disableSkeleton()
{
	_bEnableSkeleton = false;
}

//--------------------------------------------------------------
void CTKinectV2::enableColorFrame()
{
	_bEnableColorFrame = true;
}

//--------------------------------------------------------------
void CTKinectV2::disableColorFrame()
{
	_bEnableColorFrame = false;
}

//--------------------------------------------------------------
void CTKinectV2::updateBodyIndex()
{
	if(_pBodyIndexFrameReader == nullptr)
	{
		return;
	}

	//Get body index frame
	IBodyIndexFrame* pBodyIndexFrame_ = nullptr;
	if( !this->checkFunc(_pBodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame_), __FILE__, __LINE__) )
	{
		this->releaseKinectObj(pBodyIndexFrame_);
		return;
	}

	//Get body index frame buffer
	UINT iBodyIndexBufferSize_ = 0;
	BYTE* pBodyIndexBuffer_ = nullptr;
	if( !this->checkFunc(pBodyIndexFrame_->AccessUnderlyingBuffer(&iBodyIndexBufferSize_, &pBodyIndexBuffer_), __FILE__, __LINE__))
	{
		this->releaseKinectObj(pBodyIndexFrame_);
		return;
	}

	//Is new body index
	_bIsNewBodyIndex = true;

	//Get nearest user body index
	bool bHaveNearestUser_ = false;
	if(_bEnableSkeleton && _bHaveUser)
	{
		ofVec2f Joint_ = _Skeleton.aJoints[JointType::JointType_SpineMid];
		for(int iShiftY_ = -2; iShiftY_ <= 2; ++iShiftY_)
		{
			for(int iShiftX_ = -2; iShiftX_ <= 2; ++iShiftX_)
			{
				int iIndex_ = int(Joint_.y + iShiftY_) * cDEPTH_WIDTH + int(Joint_.x + iShiftX_);
				if(iBodyIndexBufferSize_ > iIndex_ && pBodyIndexBuffer_[iIndex_] != 255)
				{
					_BodyIndex = pBodyIndexBuffer_[iIndex_];
					bHaveNearestUser_ = true;
					break;
				}
			}
		}
	}

	ofPixels TmpDisplay_;
	TmpDisplay_.allocate(cDEPTH_WIDTH, cDEPTH_HEIGHT, ofImageType::OF_IMAGE_COLOR);

	//Process body index information
	unsigned char* acBodyIndexData_ = TmpDisplay_.getPixels();
	//unsigned char* acBodyIndexData_ = _BodyIndexPixels.getPixels();
	for(int iBufferIndex_ = 0; iBufferIndex_ < iBodyIndexBufferSize_; ++iBufferIndex_)
	{
		if(!_bEnableSkeleton)
		{
			if(pBodyIndexBuffer_[iBufferIndex_] != 255)
			{
				acBodyIndexData_[iBufferIndex_ * 3] = pBodyIndexBuffer_[iBufferIndex_] * 30 % 255;
				acBodyIndexData_[iBufferIndex_ * 3 + 1] = pBodyIndexBuffer_[iBufferIndex_] * 40 % 255;
				acBodyIndexData_[iBufferIndex_ * 3 + 2] = pBodyIndexBuffer_[iBufferIndex_] * 20 % 255;
			}
			else
			{
				acBodyIndexData_[iBufferIndex_ * 3] = 0;
				acBodyIndexData_[iBufferIndex_ * 3 + 1] = 0;
				acBodyIndexData_[iBufferIndex_ * 3 + 2] = 0;
			}
		}
		else
		{
			if(bHaveNearestUser_ && pBodyIndexBuffer_[iBufferIndex_] == _BodyIndex)
			{
				acBodyIndexData_[iBufferIndex_ * 3] = 249;
				acBodyIndexData_[iBufferIndex_ * 3 + 1] = 173;
				acBodyIndexData_[iBufferIndex_ * 3 + 2] = 70;
			}
			else
			{
				acBodyIndexData_[iBufferIndex_ * 3] = 0;
				acBodyIndexData_[iBufferIndex_ * 3 + 1] = 0;
				acBodyIndexData_[iBufferIndex_ * 3 + 2] = 0;
			}
		}
	}
	_BodyIndexPixels.setFromPixels(acBodyIndexData_, cDEPTH_WIDTH, cDEPTH_HEIGHT, OF_IMAGE_COLOR);


	////release buffer
	//if(pBodyIndexBuffer_ != nullptr)
	//{
	//	delete [] pBodyIndexBuffer_;
	//}

	//release frame
	this->releaseKinectObj(pBodyIndexFrame_);

}

//--------------------------------------------------------------
void CTKinectV2::updateSkeleton()
{
	if(_pBodyFrameReader == nullptr)
	{
		return;
	}

	//Get Body frame
	IBodyFrame* pBodyFrame_ = nullptr;
	if( !this->checkFunc(_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame_), __FILE__, __LINE__) )
	{
		this->releaseKinectObj(pBodyFrame_);
		return;
	}

	IBody* pBodies_[BODY_COUNT] = {0};
	if ( this->checkFunc(pBodyFrame_->GetAndRefreshBodyData(_countof(pBodies_), pBodies_), __FILE__, __LINE__) )
	{
		//Found nearest user
		int Index_ = -1;
		float fTmpDist_ = numeric_limits<float>::max();

		for(int idx_ = 0; idx_ < BODY_COUNT; ++idx_)
		{	
			BOOLEAN bTrack_ = false;
			UINT64 uint64TrackID_ = 0;

			//check body pointer
			if(!pBodies_[idx_])
			{
				continue;
			}

			//check body is tracking or not
			pBodies_[idx_]->get_IsTracked(&bTrack_);
			pBodies_[idx_]->get_TrackingId(&uint64TrackID_);
			if(!bTrack_)
			{
				continue;
			}

			//is this body is closer?
			Joint joints_[JointType_Count];
			pBodies_[idx_]->GetJoints(JointType_Count, joints_);
			
			if(joints_[JointType_SpineBase].Position.Z >= fTmpDist_)
			{
				continue;
			}

			fTmpDist_ = joints_[JointType_SpineBase].Position.Z;
			Index_ = idx_;

			//Get Left hand state
			TrackingConfidence Confidence_;
			pBodies_[idx_]->get_HandLeftConfidence(&Confidence_);
			_Skeleton.eLeftHandState = HandState_Unknown;
			if(Confidence_ == TrackingConfidence_High)
			{
				pBodies_[idx_]->get_HandLeftState(&_Skeleton.eLeftHandState);
			}
			
			//Get Right hand state
			pBodies_[idx_]->get_HandRightConfidence(&Confidence_);
			_Skeleton.eRightHandState = HandState_Unknown;
			if(Confidence_ == TrackingConfidence_High)
			{
				pBodies_[idx_]->get_HandRightState(&_Skeleton.eRightHandState);
			}

			//Get joints position
			for(int JointIdx_ = 0; JointIdx_ < JointType_Count; ++JointIdx_)
			{
				ofVec2f Pos_;
				DepthSpacePoint	DepthPoint_ = {0};
				_pCoordinateMapper->MapCameraPointToDepthSpace(joints_[JointIdx_].Position, &DepthPoint_);
				Pos_.x = DepthPoint_.X;
				Pos_.y = DepthPoint_.Y;

				_Skeleton.aJoints[JointIdx_] = Pos_;
			}
			_Skeleton.bIsTracking = true;
			_Skeleton.ui64TrackingID = uint64TrackID_;
		}

		_bHaveUser = (Index_ != -1);
	}

	//release body
	for(auto pBody_ : pBodies_)
	{
		this->releaseKinectObj(pBody_); 
	}

	//release frame
	this->releaseKinectObj(pBodyFrame_);
}

//--------------------------------------------------------------
void CTKinectV2::updateColorFrame()
{
	if(_pColorFrameReader == nullptr)
	{
		return;
	}
	
	IColorFrame* pColorFrame_ = nullptr;
	//Get Color Frame
	if( !this->checkFunc(_pColorFrameReader->AcquireLatestFrame(&pColorFrame_), __FILE__, __LINE__))
	{
		this->releaseKinectObj(pColorFrame_);
		return;
	}

	//Get image format
	ColorImageFormat ImgFormat_ = ColorImageFormat_None;
	UINT iColorBufferSize_ = 0;
	BYTE* pColorBuffer_ = nullptr;
	if( !this->checkFunc(pColorFrame_->get_RawColorImageFormat(&ImgFormat_), __FILE__, __LINE__))
	{
		this->releaseKinectObj(pColorFrame_);
		return;
	}

	if(ImgFormat_ == ColorImageFormat_Rgba)
	{
		pColorFrame_->AccessRawUnderlyingBuffer(&iColorBufferSize_, &pColorBuffer_);
	}
	else
	{
		iColorBufferSize_ = cCOLOR_WIDTH * cCOLOR_HEIGHT * cCOLOR_CHANNEL;
		pColorBuffer_ = new BYTE[iColorBufferSize_];
		pColorFrame_->CopyConvertedFrameDataToArray(iColorBufferSize_, pColorBuffer_, ColorImageFormat_Rgba);
	}

	//Is new frame
	_bIsNewColorFrame = true;

	//Processing color frame
	auto acColorFrameData_ = _ColorFramePixels.getPixels();
	for(int idx_ = 0; idx_ < (cCOLOR_WIDTH * cCOLOR_HEIGHT); ++idx_)
	{
		acColorFrameData_[idx_ * 3] = pColorBuffer_[idx_ * cCOLOR_CHANNEL];
		acColorFrameData_[idx_ * 3 + 1] = pColorBuffer_[idx_ * cCOLOR_CHANNEL + 1];
		acColorFrameData_[idx_ * 3 + 2] = pColorBuffer_[idx_ * cCOLOR_CHANNEL + 2];
	}

	//Release
	this->releaseKinectObj(pColorFrame_);
	if(pColorBuffer_ != nullptr)
	{
		delete [] pColorBuffer_;
	}
}