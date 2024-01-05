#include "LidarSimModule.h"

THIRD_PARTY_INCLUDES_START
#include <pcl/pcl_config.h>
THIRD_PARTY_INCLUDES_END


DECLARE_LOG_CATEGORY_EXTERN(LidarSimModule, Log, All);
DEFINE_LOG_CATEGORY(LidarSimModule);

void FLidarSimModule::StartupModule() {

	UE_LOG(LidarSimModule, Log, TEXT("Successfully loaded PCL Version: %s \n"), TEXT(PCL_VERSION_PRETTY));

}
void FLidarSimModule::ShutdownModule() {}


 //IMPLEMENT_MODULE(FLidarSimModule, LidarSimModule)
extern "C" __declspec(dllexport) IModuleInterface * InitializeModule() {
    return new FLidarSimModule();
}
extern "C" void IMPLEMENT_MODULE_LidarSimModule() { } uint8** GNameBlocksDebug = FNameDebugVisualizer::GetBlocks(); FChunkedFixedUObjectArray*& GObjectArrayForDebugVisualizers = GCoreObjectArrayForDebugVisualizers; UE::CoreUObject::Private::FStoredObjectPathDebug*& GComplexObjectPathDebug = GCoreComplexObjectPathDebug; UE::CoreUObject::Private::FObjectHandlePackageDebugData*& GObjectHandlePackageDebug = GCoreObjectHandlePackageDebug; 
void* operator new (size_t Size) {
    return FMemory::Malloc(Size ? Size : 1, 16ull);
} 
void* operator new[](size_t Size) {
    return FMemory::Malloc(Size ? Size : 1, 16ull);
}
void* operator new (size_t Size, const std::nothrow_t&) throw() {
    return FMemory::Malloc(Size ? Size : 1, 16ull);
}
void* operator new[](size_t Size, const std::nothrow_t&) throw() {
    return FMemory::Malloc(Size ? Size : 1, 16ull);
}
void* operator new (size_t Size, std::align_val_t Alignment) {
    return FMemory::Malloc(Size ? Size : 1, (std::size_t)Alignment);
}
void* operator new[](size_t Size, std::align_val_t Alignment) {
    return FMemory::Malloc(Size ? Size : 1, (std::size_t)Alignment);
}
void* operator new (size_t Size, std::align_val_t Alignment, const std::nothrow_t&) throw() {
    return FMemory::Malloc(Size ? Size : 1, (std::size_t)Alignment);
}
void* operator new[](size_t Size, std::align_val_t Alignment, const std::nothrow_t&) throw() {
    return FMemory::Malloc(Size ? Size : 1, (std::size_t)Alignment);
}
void operator delete (void* Ptr) {
    FMemory::Free(Ptr);
}
void operator delete[](void* Ptr) {
    FMemory::Free(Ptr);
}
void operator delete (void* Ptr, const std::nothrow_t&) throw() {
    FMemory::Free(Ptr);
}
void operator delete[](void* Ptr, const std::nothrow_t&) throw() {
    FMemory::Free(Ptr);
}
void operator delete (void* Ptr, size_t Size) {
    FMemory::Free(Ptr);
}
void operator delete[](void* Ptr, size_t Size) {
    FMemory::Free(Ptr);
}
void operator delete (void* Ptr, size_t Size, const std::nothrow_t&) throw() {
    FMemory::Free(Ptr);
}
void operator delete[](void* Ptr, size_t Size, const std::nothrow_t&) throw() {
    FMemory::Free(Ptr);
}
void operator delete (void* Ptr, std::align_val_t Alignment) {
    FMemory::Free(Ptr);
}
void operator delete[](void* Ptr, std::align_val_t Alignment) {
    FMemory::Free(Ptr);
}
void operator delete (void* Ptr, std::align_val_t Alignment, const std::nothrow_t&) throw() {
    FMemory::Free(Ptr);
}
void operator delete[](void* Ptr, std::align_val_t Alignment, const std::nothrow_t&) throw() {
    FMemory::Free(Ptr);
}
void operator delete (void* Ptr, size_t Size, std::align_val_t Alignment) {
    FMemory::Free(Ptr);
}
void operator delete[](void* Ptr, size_t Size, std::align_val_t Alignment) {
    FMemory::Free(Ptr);
}
void operator delete (void* Ptr, size_t Size, std::align_val_t Alignment, const std::nothrow_t&) throw() {
    FMemory::Free(Ptr);
}
void operator delete[](void* Ptr, size_t Size, std::align_val_t Alignment, const std::nothrow_t&) throw() {
    FMemory::Free(Ptr); 
}
void* StdMalloc(size_t Size, size_t Alignment) {
    return FMemory::Malloc(Size ? Size : 1, Alignment);
}
void* StdRealloc(void* Original, size_t Size, size_t Alignment) {
    return FMemory::Realloc(Original, Size ? Size : 1, Alignment);
}
void StdFree(void* Ptr) {
    FMemory::Free(Ptr);
}


//#include <exception>
//THIRD_PARTY_INCLUDES_START
//#include <boost/throw_exception.hpp>
//THIRD_PARTY_INCLUDES_END
//
//namespace boost {
//	void boost::throw_exception(class std::exception const&) {}
//}