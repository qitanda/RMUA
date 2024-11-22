
#include <stdlib.h>
#include <common/ilogger.hpp>

using namespace std;

bool onnx_hub(const char* name, const char* save_to) {

	auto onnx_file = iLogger::format("%s.onnx", name);
	if (!iLogger::exists(onnx_file)) {
		INFO("Auto download %s", onnx_file.c_str());
		system(iLogger::format("wget http://zifuture.com:1556/fs/25.shared/%s --output-document=%s", onnx_file.c_str(), save_to).c_str());
	}

	bool exists = iLogger::exists(save_to);
	if (!exists) {
		INFOE("Download %s failed", onnx_file.c_str());
	}
	return exists;
}