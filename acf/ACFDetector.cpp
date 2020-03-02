/*
 * ACFDetector.cpp
 */

#include <sstream>
#include <chrono>
#include <tbb/tbb.h>
#include <matio.h>
#include "../general/NonMaximumSuppression.h"

#include "ACFDetector.h"
#include "ACFFeaturePyramid.h"

#define USE_TBB

static inline void getChild(float *chns1, uint32_t *cids, uint32_t *fids,
        float *thrs, uint32_t offset, uint32_t &k0, uint32_t &k) {
    float ftr = chns1[cids[fids[k]]];
    k = (ftr < thrs[k]) ? 1 : 2;
    k0 = k += k0 * 2;
    k += offset;
}

DetectionList ACFDetector::applyDetector(const cv::Mat &Frame) {

    auto measure_time = std::chrono::high_resolution_clock::now();

    // 计算特征金字塔
    if (feature_pyramid) {
        delete feature_pyramid;
        feature_pyramid = NULL;
    }
    feature_pyramid = new ACFFeaturePyramid(Frame, 8,
            cv::Size(this->model_width, this->model_height), this->shrinking,
            this->lambdas, this->pad_width, this->pad_height);

    calc_feature_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - measure_time).count();

//    feature_pyramid->print_duration();

    // 验证用于在估计尺度下计算特征图的幂法则
//    for (size_t i = 0, x = 0; i < pACFP->getAmount(); i += 2) {
//        pACFP->getLayer(i)->showFeature(4, "pic" + std::to_string(i), x, 400);
////        pACFP->getLayer(i)->saveFeature(4, "/home/pi/features/L" + std::to_string(i) + "F4.png");
//        x += pACFP->getLayer(i)->getChannelWidth() + 4;
//    }

// 验证特征计算
//    for (size_t i = 0, x = 0; i < pACFP->getLayer(0)->getnChannels(); i += 1) {
//        pACFP->getLayer(0)->showFeature(i, "pic" + std::to_string(i), x, 400);
//        x += pACFP->getLayer(0)->getChannelWidth() + 4;
//    }

//    return DetectionList();

    measure_time = std::chrono::high_resolution_clock::now();

    // use tbb to detect, save about 20 ms at 320x240 (serial_for cost 30ms)
    // 对每个尺度分别调用一次滑动窗口检测
    tbb::concurrent_vector<Detection> det_temp;
#ifdef USE_TBB
    tbb::parallel_for(size_t(0), size_t(feature_pyramid->getAmount()),
            [&](size_t layer_i) {
#else
    for (size_t i = 0; i < feature_pyramid->getAmount(); i++) {
#endif
        auto layer = feature_pyramid->getLayer(layer_i);
        if (layer != NULL) {
            std::vector<Detection> det = Detect(layer);
            for (int i = 0; i < det.size(); i++) {
                // 根据缩放尺度, 修改检测结果尺寸和位置
                cv::Size2d scale_xy = feature_pyramid->get_scale_xy(layer_i);
                det[i].setX(det[i].getX() / scale_xy.width);
                det[i].setY(det[i].getY() / scale_xy.height);
                det[i].setWidth(det[i].getWidth() / scale_xy.width);
                det[i].setHeight(det[i].getHeight() / scale_xy.height);
                det[i].setColor(cv::Scalar(0, 0, 255));
                det_temp.push_back(det[i]);
            }
        } else {
            std::cout << "layer " << layer_i << " is NULL!" << std::endl;
        }
#ifdef USE_TBB
    });
#else
    }
#endif

    DetectionList DL;
    // 添加到检测结果中DetectionList
    for (int i = 0; i < det_temp.size(); i++) {
        DL.addDetection(det_temp[i]);
    }
    apply_classifier_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - measure_time).count();

    return DL;
}

std::vector<Detection> ACFDetector::Detect(
        const ChannelFeatures *features) const {
    std::vector<Detection> dets;

//    float cascThr = -1; //could also come from model
    float cascThr = this->cascThr; //could also come from model
    int stride = this->shrinking;
    int shrink = this->shrinking;
    float* chns = features->chns;

    int treeDepth = this->ModelDepth;
    int chnWidth = features->getChannelWidth();
    int chnHeight = features->getChannelHeight();
    int width = chnWidth;                // 积分特征图的宽度
    int height = chnHeight;               // 积分特征图的高度

    int nTrees = this->nTrees;
    int nTreeNodes = this->nTreeNodes;

    // Should be kept in the model
    int modelWd = this->model_width_pad;
    int modelHt = this->model_height_pad;

    //Height and width of the area to cover with the sliding window-detector
    int height1 = static_cast<int>(std::ceil(
            static_cast<float>(chnHeight * shrinking - modelHt + 1) / stride));
    int width1 = static_cast<int>(std::ceil(
            static_cast<float>(chnWidth * shrinking - modelWd + 1) / stride));

    int nChns = features->getnChannels();

    // construct cids array 构造cids数组, 该数组用于将(窗口位置+区域位置)映射到原始特征图
    int nFtrs = modelHt / shrink * modelWd / shrink * nChns; // 每个检测窗口中的总特征数量 32/2*32/2*10
    uint32_t *cids = new uint32_t[nFtrs];                     // 创建通道索引数组
    int m = 0;                                             // 组织方式: 列->行->面(通道)
    for (int z = 0; z < nChns; z++)                        // 遍历特征通道
        for (int c = 0; c < modelWd / shrink; c++)           // 遍历宽度
            for (int r = 0; r < modelHt / shrink; r++)         // 遍历高度
                cids[m++] = z * width * height + c * height + r; // 设置索引号

            // apply classifier to each patch
    tbb::concurrent_vector<int> rs, cs;
    tbb::concurrent_vector<float> hs1;
    tbb::concurrent_vector<int> levels;

    auto measure_time = std::chrono::high_resolution_clock::now();

    // 遍历减采样后的宽度和高度
    // 使用并行遍历, 最多可减少50%的时间
#ifdef USE_TBB
    tbb::parallel_for(size_t(0), size_t(width1), [&](size_t c) {
#else
    for (int c = 0; c < width1; c++) {
#endif
        // 遍历Y轴
        for (int r = 0; r < height1; r++) {
            float h = 0;
            int t;
            // 获取对应坐标位置的通道数据
            float *chns1 = chns + (r * stride / shrink)
                    + (c * stride / shrink) * height;
            if (treeDepth == 1) {
                // specialized case for treeDepth==1
                for (t = 0; t < nTrees; t++) {
                    uint32_t offset = t * nTreeNodes, k = offset, k0 = 0;
                    getChild(chns1, cids, fids, thrs, offset, k0, k);
                    h += hs[k];
                    if (h <= cascThr)
                        break;
                }
            } else if (treeDepth == 2) {
                // specialized case for treeDepth==2
                for (t = 0; t < nTrees; t++) {
                    uint32_t offset = t * nTreeNodes, k = offset, k0 = 0;
                    getChild(chns1, cids, fids, thrs, offset, k0, k);
                    getChild(chns1, cids, fids, thrs, offset, k0, k);
                    h += hs[k];
                    if (h <= cascThr)
                        break;
                }
            } else if (treeDepth > 2) {
                // specialized case for treeDepth>2
                for (t = 0; t < nTrees; t++) {
                    uint32_t offset = t * nTreeNodes, k = offset, k0 = 0;
                    for (int i = 0; i < treeDepth; i++)
                        getChild(chns1, cids, fids, thrs, offset, k0, k);
                    h += hs[k];
                    if (h <= cascThr)
                        break;
                }
            } else {
                // general case (variable tree depth), 遍历4096个弱分类器(决策树)
                for (t = 0; t < nTrees; t++) {
                    uint32_t offset = t * nTreeNodes;
                    // k为当前节点号
                    uint32_t k = offset;
                    // child[k]为节点k对应的子节点的索引号
                    while (child[k]) {
                        // fids[k]        待对比特征在采样窗口中的偏移量
                        // cids[fids[k]]  待对比特征在特征图中的偏移量
                        float ftr = chns1[cids[fids[k]]];             // 为待对比的特征
                        // 根据特征值与给定阈值的大小关系, 转到对应的子节点
                        k = child[k] - ((ftr < thrs[k]) ? 1 : 0) + offset;
                    }
                    h += hs[k];
                    if (h <= cascThr)
                        break; // 如果评分低于阈值, 则立即停止判断
                }
            }
            // 如果该窗口评分大于阈值, 则记录该窗口的位置和置信度
            if (h > cascThr) {
                cs.push_back(c);
                rs.push_back(r);
                hs1.push_back(h);
                levels.push_back(t);
            }
        }
#ifdef USE_TBB
    });
#else
    }
#endif

    delete[] cids;

    float shiftw = (this->model_width_pad - this->model_width) / 2.0; // when padding is used, this should also be subtracted ...
    float shifth = (this->model_height_pad - this->model_height) / 2.0; // "

    for (int i = 0; i < cs.size(); i++) {
        Detection det;
        det.setX((cs[i]) * shrinking - this->pad_width + shiftw);
        det.setY((rs[i]) * shrinking - this->pad_height + shifth);
        det.setWidth(this->model_width);
        det.setHeight(this->model_height);
        det.setScore(hs1[i]);
        det.setLevel(levels[i]);
        dets.push_back(det);
    }

    return dets;
}

ACFDetector::ACFDetector(std::string modelfile) {
    ReadModel(modelfile);
}

// 读取检测器模型
void ACFDetector::ReadModel(std::string modelfile) {

    if (modelfile.substr(modelfile.size() - 4, 4) == ".mat") {
        const char *detector_path = modelfile.c_str();
        std::cout << "Loading detector(" << detector_path << ")..."
                << std::flush;
        mat_t *matfp = Mat_Open(detector_path, MAT_ACC_RDONLY);
        if (matfp) {
            matvar_t *detector = NULL;
            matvar_t *opt = NULL;
            matvar_t *pPyramid = NULL;
            matvar_t *pChns = NULL;
            matvar_t *pad = NULL;
            matvar_t *lambdas = NULL;
            matvar_t *shrink = NULL;
            matvar_t *modelDs = NULL;
            matvar_t *modelDsPad = NULL;
            matvar_t *cascThr = NULL;
            matvar_t *name = NULL;
            matvar_t *clf = NULL;
            matvar_t *treeDepth = NULL;
            matvar_t *fids = NULL;
            matvar_t *thrs = NULL;
            matvar_t *child = NULL;
            matvar_t *hs = NULL;

            // 读取并打印检测器名称
            detector = Mat_VarRead(matfp, "detector");
            if (detector) {
                opt = Mat_VarGetStructFieldByName(detector, "opts", 0);
                if (opt) {
                    name = Mat_VarGetStructFieldByName(opt, "name", 0);
                    modelDs = Mat_VarGetStructFieldByName(opt, "modelDs", 0);
                    modelDsPad = Mat_VarGetStructFieldByName(opt, "modelDsPad",
                            0);
                    cascThr = Mat_VarGetStructFieldByName(opt, "cascThr", 0);
                    pPyramid = Mat_VarGetStructFieldByName(opt, "pPyramid", 0);
                    if (pPyramid) {
                        pad = Mat_VarGetStructFieldByName(pPyramid, "pad", 0);
                        lambdas = Mat_VarGetStructFieldByName(pPyramid,
                                "lambdas", 0);
                        pChns = Mat_VarGetStructFieldByName(pPyramid, "pChns",
                                0);
                        if (pChns) {
                            shrink = Mat_VarGetStructFieldByName(pChns,
                                    "shrink", 0);
                        }
                    }
                }
                // 读取分类器数据
                clf = Mat_VarGetStructFieldByName(detector, "clf", 0);
                if (clf) {
                    treeDepth = Mat_VarGetStructFieldByName(clf, "treeDepth",
                            0);
                    fids = Mat_VarGetStructFieldByName(clf, "fids", 0);
                    thrs = Mat_VarGetStructFieldByName(clf, "thrs", 0);
                    child = Mat_VarGetStructFieldByName(clf, "child", 0);
                    hs = Mat_VarGetStructFieldByName(clf, "hs", 0);
                }
            }

            if (name && shrink && modelDs && modelDsPad && cascThr && fids
                    && thrs && child && hs && pad && lambdas) {
                // 模型文件读取成功
                std::cout << " OK" << std::endl;

                // 从MAT对象中提取数据
                const char *detector_name = (const char *) name->data;
                double detector_shrink = ((const double*) shrink->data)[0];
                const double *detector_modelDs = (const double*) modelDs->data;
                const double *detector_modelDsPad =
                        (const double*) modelDsPad->data;
                double *detector_lambdas = (double*) lambdas->data;
                double *detector_pad = (double*) pad->data;
                double detector_cascThr = ((const double*) cascThr->data)[0];
                uint32_t detector_treeDepth =
                        treeDepth ? ((const uint32_t*) treeDepth->data)[0] : 0;
                uint32_t detector_nNodes = fids->dims[0];
                uint32_t detector_nWeaks = fids->dims[1];
                const uint32_t *detector_fids = (const uint32_t *) fids->data;
                const float *detector_thrs = (const float *) thrs->data;
                const uint32_t *detector_child = (const uint32_t *) child->data;
                const float *detector_hs = (const float *) hs->data;

                // 打印检测器信息
                std::cout << "Detector Name:        " << detector_name
                        << std::endl;
                std::cout << "Detector modelDs:     " << detector_modelDs[0]
                        << "x" << detector_modelDs[1] << std::endl;
                std::cout << "Detector modelDsPad:  " << detector_modelDsPad[0]
                        << "x" << detector_modelDsPad[1] << std::endl;
                std::cout << "Detector lambdas:     " << detector_lambdas[0]
                        << "," << detector_lambdas[1] << ","
                        << detector_lambdas[2] << std::endl;
                std::cout << "Detector pad:         " << detector_pad[0] << ","
                        << detector_pad[1] << std::endl;
                std::cout << "Detector shrink:      " << detector_shrink
                        << std::endl;
                std::cout << "Detector cascThr:     " << detector_cascThr
                        << std::endl;
                std::cout << "Detector treeDepth:   " << detector_treeDepth
                        << std::endl;
                std::cout << "Detector classifier:  " << detector_nNodes << "x"
                        << detector_nWeaks << std::endl;

                // 将检测器数据拷贝至该对象
                std::cout << "Copying data...";
                this->setHeight(detector_modelDs[0]);
                this->setWidth(detector_modelDs[1]);
                this->setHeightPad(detector_modelDsPad[0]);
                this->setWidthPad(detector_modelDsPad[1]);
                this->pad_height = detector_pad[0];
                this->pad_width = detector_pad[1];
                this->lambdas = {detector_lambdas[0],
                    detector_lambdas[1], detector_lambdas[2]};
                this->ModelDepth = detector_treeDepth;
                this->shrinking = detector_shrink;
                this->cascThr = detector_cascThr;
                this->nTrees = detector_nWeaks;
                this->nTreeNodes = detector_nNodes;

                this->child = new uint32_t[detector_nNodes * detector_nWeaks];
                this->fids = new uint32_t[detector_nNodes * detector_nWeaks];
                this->thrs = new float[detector_nNodes * detector_nWeaks];
                this->hs = new float[detector_nNodes * detector_nWeaks];

                memcpy(this->child, detector_child,
                        sizeof(uint32_t) * detector_nNodes * detector_nWeaks);
                memcpy(this->fids, detector_fids,
                        sizeof(uint32_t) * detector_nNodes * detector_nWeaks);
                memcpy(this->thrs, detector_thrs,
                        sizeof(float) * detector_nNodes * detector_nWeaks);
                memcpy(this->hs, detector_hs,
                        sizeof(float) * detector_nNodes * detector_nWeaks);

                this->Child = new const uint32_t*[detector_nWeaks];	// 创建指针数组
                this->Fid = new const uint32_t*[detector_nWeaks];	// 创建指针数组
                this->Thresholds = new const float*[detector_nWeaks];// 创建指针数组
                this->Values = new const float*[detector_nWeaks];	// 创建指针数组
                for (int i = 0; i < detector_nWeaks; i++) {
                    this->Child[i] = &(this->child[i * detector_nNodes]);
                    this->Fid[i] = &(this->fids[i * detector_nNodes]);
                    this->Thresholds[i] = &(this->thrs[i * detector_nNodes]);
                    this->Values[i] = &(this->hs[i * detector_nNodes]);
                }

                std::cout << " OK" << std::endl;
            } else {
                // MAT文件数据格式错误
                std::cout << "MAT file wrong format" << std::endl;
            }
            Mat_Close(matfp);
        } else {
            // 打开MAT文件失败
            std::cout << "Error opening MAT file" << std::endl;
        }

    } else {
        std::cout << "Unsupported model" << std::endl;
    }
}

ACFDetector::~ACFDetector() {
    free(this->thrs);
    free(this->hs);
    free(this->fids);
    free(this->child);
    if (feature_pyramid) {
        delete feature_pyramid;
        feature_pyramid = NULL;
    }
}

