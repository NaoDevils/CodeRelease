#pragma once
extern int interpolationFrameCounter;


#define DECLARE_INTERPOLATE_VAR(varname, type, initialValue) \
	inline type interpolate##varname(type val, float maxDiff, bool reset = false) \
{ \
	static type old = initialValue;\
	static int oldCount=0;\
\
  if (reset)\
  {\
    old = initialValue;\
    oldCount = 0;\
    return initialValue;\
  }\
\
	if (interpolationFrameCounter>oldCount)\
	{\
		if (val-old>maxDiff) val=old+maxDiff;\
		if (val-old<-maxDiff) val=old-maxDiff;\
		old=val;\
		oldCount=interpolationFrameCounter;\
	}\
	return old;\
}

#define RESET_INTERPOLATION_VAR(varname) \
  interpolate##varname(0, 0, true);

#define INTERPOLATE_VAR(name, val, maxDiff) \
	interpolate##name(val, maxDiff)


#define DECLARE_INTERPOLATE(varname, type, maxDiff, initialValue) \
	inline type interpolate##varname(type val, bool reset = false) \
{ \
  static type old = initialValue;\
	static int oldCount=0;\
  \
  if (reset)\
  {\
    old = initialValue;\
    oldCount = 0;\
    return initialValue;\
  }\
\
	if (interpolationFrameCounter>oldCount)\
	{\
		if (val-old>maxDiff) val=old+maxDiff;\
		if (val-old<-maxDiff) val=old-maxDiff;\
		old=val;\
		oldCount=interpolationFrameCounter;\
	}\
	return old;\
}

#define INTERPOLATE(name, val) \
	interpolate##name(val)

#define RESET_INTERPOLATION(varname) \
  interpolate##varname(0, true);

#define INTERPOLATE_AND_STORE(name, val) \
	name=interpolate##varname(val)

#define DECLARE_INTERPOLATE_ARRAY(varname, type, maxDiff, size) \
	inline type interpolate##varname(type val, int index) \
{ \
	static type old[size];\
	static bool newFrame[size];\
	static int oldCount=0;\
	if (interpolationFrameCounter>oldCount) {\
		for (int i=0; i<size; i++) newFrame[i]=true;\
		oldCount=interpolationFrameCounter;\
	}\
	if (newFrame[index])\
	{\
		if (val-old[index]>maxDiff) val=old[index]+maxDiff;\
		if (val-old[index]<-maxDiff) val=old[index]-maxDiff;\
		old[index]=val;\
		newFrame[index]=false;\
	}\
	return old[index];\
}

#define INTERPOLATE_ARRAY_ELEMENT(name, val, index) \
	interpolate##name(val, index)

#define INTERPOLATE_ARRAY_ELEMENT_AND_STORE(name, val, index) \
	val=interpolate##name(val, index)

#define INTERPOLATE_AND_STORE_ARRAY(name, val, size) \
	for (int intpolateindex=0; intpolateindex<size; intpolateindex++)\
		val[intpolateindex]=interpolate##name(val[intpolateindex], intpolateindex)
