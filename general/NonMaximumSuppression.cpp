
#include "NonMaximumSuppression.h"

NonMaximumSuppression::NonMaximumSuppression() {
}

NonMaximumSuppression::~NonMaximumSuppression() {
}

float calcOverlapArea(Detection &i, Detection &j) {
    float overlap_width = std::min(i.getX() + i.getWidth(),
            j.getX() + j.getWidth()) - std::max(i.getX(), j.getX());
    float overlap_height = std::min(i.getY() + i.getHeight(),
            j.getY() + j.getHeight()) - std::max(i.getY(), j.getY());
    // 检查是否重合
    if (overlap_width <= 0 || overlap_height <= 0) {
        return 0;
    } else {
        return overlap_width * overlap_height;
    }
}

DetectionList NonMaximumSuppression::dollarNMS(DetectionList &DL_in) {
    // 重合面积阈值
    float overlap_threshold = 0.65;

    // 按得分从大到小排序
    std::sort(DL_in.detections.begin(), DL_in.detections.end(),
            [](const Detection &i, const Detection &j) {
                return (i.getScore() > j.getScore());
            });

    std::vector<float> area(DL_in.detections.size());     // 节点面积
    // 计算每一个检测结果的面积
    for (int i = 0; i < DL_in.detections.size(); i++) {
        area[i] = DL_in.detections[i].getWidth()
                * DL_in.detections[i].getHeight();
    }

    std::vector<bool> kp(DL_in.detections.size(), true);  // 当前节点是否已被剔除标志
    for (int i = 0; i < DL_in.detections.size(); i++) {
        // 检查当前节点i是否已被剔除
        if (kp[i] == false) {
            continue;
        }
        // 遍历得分低于节点i的所有节点
        for (int j = i + 1; j < DL_in.detections.size(); j++) {
            // 检查当前节点j是否已被剔除
            if (kp[j] == false) {
                continue;
            }
            // 计算重合区域面积
            float overlap_area = calcOverlapArea(DL_in.detections[i], DL_in.detections[j]);
            // 若重合区域面积/两个检测结果中面积较小的那个<重合阈值
            if (overlap_area / std::min(area[i], area[j])
                    > overlap_threshold) {
                // 剔除该节点
                kp[j] = false;
            }

        }
    }

    std::vector<int> father(DL_in.detections.size(), -1);
    for (int i = 0; i < DL_in.detections.size(); i++) {
        if (kp[i] == true) {
            father[i] = i;
        } else {
            float max_overlap = 0;
            for (int j = 0; j < DL_in.detections.size(); j++) {
                if (kp[j] == true) {
                    float overlap_area = calcOverlapArea(DL_in.detections[i], DL_in.detections[j]);
                    if (overlap_area > max_overlap) {
                        max_overlap = overlap_area;
                        father[i] = j;
                    }
                }
            }
            assert(father[i] != -1);
        }
    }

    // 将剩余未被剔除的节点放入新的检测结果列表
    DetectionList DL_out;
    for (int i = 0; i < DL_in.detections.size(); i++) {
        if (kp[i] == true) {
            float sum_x = 0;
            float sum_y = 0;
            float sum_width = 0;
            float sum_height = 0;
            float sum_score = 0;
            int count_child = 0;
            for (int j = 0; j < DL_in.detections.size(); j++) {
                if (father[j] == i) {
                    sum_x += DL_in.detections[j].getX() * DL_in.detections[j].getScore();
                    sum_y += DL_in.detections[j].getY() * DL_in.detections[j].getScore();
                    sum_width += DL_in.detections[j].getWidth() * DL_in.detections[j].getScore();
                    sum_height += DL_in.detections[j].getHeight() * DL_in.detections[j].getScore();
                    sum_score += DL_in.detections[j].getScore();
                    count_child++;
                }
            }
            Detection det = Detection();
            det.setX(sum_x / sum_score);
            det.setY(sum_y / sum_score);
            det.setWidth(sum_width / sum_score);
            det.setHeight(sum_height / sum_score);
            det.setScore(DL_in.detections[i].getScore());
            det.setLevel(count_child);
            det.setColor(cv::Scalar(0, 255, 0));
            DL_out.detections.push_back(det);
        }
    }

    return DL_out;

}
