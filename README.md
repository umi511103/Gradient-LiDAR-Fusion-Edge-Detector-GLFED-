//中文

Depth-Gradient Edge Detection Algorithm (DGEA) / 深度梯度邊緣檢測算法

本演算法旨在結合光達點雲與相機影像，提取點雲中的邊緣與平面特徵，並將其有效地映射至深度影像，以減輕光照變化對特徵提取的影響。

點雲預處理：

去除無效點： 移除點雲中的NaN值，確保資料的完整性。
掃描線分割： 根據光達掃描線，將點雲分為多個子點雲，以便獨立處理。
特徵提取：

曲率計算： 針對每個掃描線上的點，計算其曲率值。曲率較大的點代表其鄰近區域的幾何形狀變化較大，通常為邊緣點；而曲率較小的點則代表其鄰近區域較為平坦，通常為平面點。
區域分割： 將掃描線劃分為多個區域，以便更精細地提取特徵。
特徵點篩選：
邊緣點： 首先選取曲率超過特定閾值的點 (本演算法中為0.06或0.3，此閾值是經實驗調整得出)。接著，檢查這些點的鄰近點是否也位於邊緣上。此步驟能有效地去除孤立的噪聲點，確保所提取的邊緣點連續且具有代表性。
平面點：
深度一致性檢查： 選取在一定鄰域範圍內 (由window_size參數控制，本演算法中為5) 深度變化較小的點。這些點通常屬於同一個平面。
梯度一致性檢查： 進一步檢查這些點在深度影像上的梯度變化是否一致。此步驟能排除深度變化雖然小，但實際上屬於斜坡或曲面的點。
影像強度一致性檢查： 若點不符合上述深度與梯度條件，則檢查其鄰域內的影像強度是否大致相同。此步驟利用影像資訊，彌補光達在平面檢測上的不足，特別是針對光照變化較大的場景。
降採樣： 對邊緣點雲與平面點雲進行降採樣，減少數據量，提升後續處理效率。
深度影像融合：

點雲投影： 將平面點投影至深度影像，並記錄深度值。
邊緣增強： 透過Sobel算子計算深度影像的梯度，以增強邊緣資訊。
邊緣點投影與優化： 將邊緣點投影至深度影像，並檢查投影點是否位於增強後的邊緣附近。此步驟利用影像的邊緣資訊，進一步確認點雲中提取的邊緣點的準確性。
平面點雲還原與優化： 利用深度影像中的深度值，將平面點從影像座標還原至三維座標，並進行降採樣優化。

由於我們將光達掃描點集中在kitti車輛的左邊相機，因此模擬也是以這範圍為主
模擬成績如下(測量使用這工具https://github.com/LeoQLi/KITTI_odometry_evaluation_tool.git) :
原算法
![image](https://github.com/user-attachments/assets/6f0a9cb5-0f67-463b-a917-b1842c64dc0f)

深度梯度邊緣檢測算法(GLFED)

![image](https://github.com/user-attachments/assets/a0404f84-f462-4ec1-9f5a-887e6e5b0fe8)

//=========================
執行此專案記得將boot刪除 ，內部為模擬bash
//=========================
使用其程式碼務必標著作者及參考作者
架構參考作者 : 
// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
作者
// National Chung Hsing University
// Author of GLFED: Jia-En Li
// Email g112002612@mail.nchu.edu.tw


//English

Depth-Gradient Edge Detection Algorithm (DGEA) / 深度梯度边缘检测算法

The purpose of this algorithm is to combine LiDAR point clouds with camera images to extract edge and plane features from the point clouds and map them effectively to depth images. This reduces the impact of lighting changes on feature extraction.

Point Cloud Preprocessing
Remove Invalid Points: Eliminate NaN values from the point cloud to ensure data integrity.
Scan Line Segmentation: Divide the point cloud into multiple sub-point clouds based on LiDAR scan lines for independent processing.
Feature Extraction
Curvature Calculation: For each point on the scan line, calculate its curvature value. Points with larger curvature indicate significant geometric shape changes in their vicinity, often representing edge points. Points with smaller curvature indicate flat areas, typically plane points.
Region Segmentation: Segment the scan lines into multiple regions for finer feature extraction.
Feature Point Selection
Edge Points:
Initially, select points with curvature exceeding a specific threshold (0.06 or 0.3, determined experimentally in this algorithm).
Then, check if the neighboring points are also on the edge. This step effectively removes isolated noise points, ensuring the extracted edge points are continuous and representative.
Plane Points:
Depth Consistency Check: Select points with minor depth variation within a certain neighborhood range, controlled by the window_size parameter (set to 5 in this algorithm). These points usually belong to the same plane.
Gradient Consistency Check: Further check whether the gradient changes of these points in the depth image are consistent. This step excludes points that have small depth changes but belong to slopes or surfaces.
Image Intensity Consistency Check: If the points do not meet the depth and gradient conditions above, check if the image intensity within their neighborhood is relatively uniform. This step utilizes image information to compensate for LiDAR's shortcomings in plane detection, especially in scenes with significant lighting variations.
Downsampling: Perform downsampling on edge and plane point clouds to reduce data volume and improve subsequent processing efficiency.
Depth Image Fusion
Point Cloud Projection: Project the plane points onto the depth image and record the depth values.
Edge Enhancement: Use the Sobel operator to calculate the gradient of the depth image to enhance edge information.
Edge Point Projection and Optimization: Project edge points onto the depth image and check if the projection points are near the enhanced edges. This step leverages the image's edge information to further confirm the accuracy of the edge points extracted from the point cloud.
Plane Point Cloud Restoration and Optimization: Use the depth values from the depth image to restore plane points from image coordinates to 3D coordinates and perform downsampling optimization.
The LiDAR scan points are concentrated on the left camera of the Kitti vehicle, so the simulation focuses on this area as well.

Simulation Results (in chinese description)

Note:

When running this project, remember to delete the boot file, as it simulates the bash environment.
If using this code, please credit the authors and reference them as follows:
Structure Reference Author:

FLOAM Author: Wang Han
Email: wh200720041@gmail.com
Homepage: https://wanghan.pro
Author of GLFED:

University: National Chung Hsing University
Author: Jia-En Li
Email: g112002612@mail.nchu.edu.tw
