#include "FAST/Data/DataObject.hpp"
#include "FAST/ProcessObject.hpp"

namespace fast {

DataObject::DataObject() :
        mTimestampModified(0),
        mTimestampCreated(0) {

    mDataIsBeingAccessed = false;
    mDataIsBeingWrittenTo = false;
}

void DataObject::blockIfBeingWrittenTo() {
    std::unique_lock<std::mutex> lock(mDataIsBeingWrittenToMutex);
    while(mDataIsBeingWrittenTo) {
        mDataIsBeingWrittenToCondition.wait(lock);
    }
}

void DataObject::blockIfBeingAccessed() {
    std::unique_lock<std::mutex> lock(mDataIsBeingAccessedMutex);
    while(mDataIsBeingAccessed) {
        mDataIsBeingAccessedCondition.wait(lock);
    }
}

void DataObject::accessFinished() {
	{
        std::unique_lock<std::mutex> lock(mDataIsBeingWrittenToMutex);
        mDataIsBeingWrittenTo = false;
        mDataIsBeingWrittenToCondition.notify_all();
	}

	{
        std::unique_lock<std::mutex> lock(mDataIsBeingAccessedMutex);
        mDataIsBeingAccessed = false;
        mDataIsBeingAccessedCondition.notify_all();
	}
}

uint64_t DataObject::getTimestamp() const {
    return mTimestampModified;
}

uint64_t DataObject::getCreationTimestamp() const {
    return mTimestampCreated;
}

void DataObject::setCreationTimestamp(uint64_t timestamp) {
    mTimestampCreated = timestamp;
}

void DataObject::updateModifiedTimestamp() {
    mTimestampModified++;
}

void DataObject::setMetadata(std::string name, std::string value) {
    mMetadata[name] = value;
}

std::string DataObject::getMetadata(std::string name) const {
    if(mMetadata.count(name) == 0)
        throw Exception("Metadata " + name + " was not found.");

    return mMetadata.at(name);
}

std::map<std::string, std::string> DataObject::getMetadata() const {
    return mMetadata;
}

void DataObject::setMetadata(std::map<std::string, std::string> metadata) {
    mMetadata = metadata;
}

void DataObject::deleteMetadata(std::string name) {
    mMetadata.erase(name);
}

void DataObject::setLastFrame(std::string streamer) {
    m_lastFrame.insert(streamer);
}

bool DataObject::isLastFrame() {
    return !m_lastFrame.empty();
}
bool DataObject::isLastFrame(std::string streamer) {
    return m_lastFrame.count(streamer) > 0;
}

std::set<std::string> DataObject::getLastFrame() {
    return m_lastFrame;
}

void DataObject::setFrameData(std::string name, std::string value) {
    m_frameData[name] = value;
}

std::string DataObject::getFrameData(std::string name) {
    if(m_frameData.count(name) == 0)
        throw Exception("Frame data " + name + " does not exist.");

    return m_frameData[name];
}

std::map<std::string, std::string> DataObject::getFrameData() {
    return m_frameData;
}

void DataObject::removeLastFrame(std::string streamer) {
    m_lastFrame.erase(streamer);
}

void DataObject::clearLastFrame() {
    m_lastFrame.clear();
}

bool DataObject::hasFrameData(std::string name) const {
    return m_frameData.count(name) > 0;
}

void DataObject::setFrameData(std::map<std::string, std::string> frameData) {
    m_frameData = frameData;
}


template <>
int DataObject::getFrameData(std::string name) {
    return std::stoi(getFrameData(name));
}
template <>
float DataObject::getFrameData(std::string name) {
    return std::stof(getFrameData(name));
}

} // end namespace fast
