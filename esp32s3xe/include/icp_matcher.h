/**
 * AMR 2.0 — ICP Scan Matcher (ESP32-S3)
 *
 * Iterative Closest Point (Point-to-Point) với closed-form 2D solution.
 * Không dùng SVD matrix — chỉ cần 4 float cho cross-covariance 2D.
 *
 * Mục đích: Hiệu chỉnh pose odometry bằng cách so sánh 2 LIDAR scan liên tiếp.
 *   prev_scan (N-1) ← reference frame
 *   curr_scan (N)   ← query — biến đổi để khớp với prev
 *   output: (dx, dy, dtheta) correction cộng vào robotX/Y/Theta
 *
 * Đặc điểm thiết kế:
 *   - Header-only, zero malloc, static buffers
 *   - Brute-force nearest-neighbor O(n²), n≤360 → ~130k phép so sánh/iter
 *   - Tối ưu cho Core 0 (LidarTask), target < 20ms/call với MAX_ITER=10
 *   - Scan buffers dùng PSRAM nếu BOARD_HAS_PSRAM được định nghĩa
 */

#ifndef ICP_MATCHER_H
#define ICP_MATCHER_H

#include <Arduino.h>
#include <cmath>
#include <cstring>
#include "lidar_mapper.h"   // LidarPoint struct

// ============================================================
//   CONFIG
// ============================================================

#define ICP_MAX_PTS      360    // Max scan points (1 vòng quét LIDAR)
#define ICP_MAX_ITER     10     // Số iteration tối đa
#define ICP_EPSILON      1e-4f  // Ngưỡng hội tụ (norm của transform)
#define ICP_MAX_DIST_M   0.4f   // Bỏ qua correspondences xa hơn 0.4m
#define ICP_MIN_PAIRS    20     // Tối thiểu 20 cặp điểm để solve

// ============================================================
//   ICP MATCHER CLASS
// ============================================================

class IcpMatcher {
public:
    // ── Data types ────────────────────────────────────────────

    struct Pose2D {
        float x;
        float theta;
        float y;
    };

    struct ScanPt {
        float x, y;
    };

    // ── Public API ────────────────────────────────────────────

    /**
     * So khớp curr_scan với prev_scan.
     *
     * @param prev       Mảng LidarPoint của scan N-1 (reference)
     * @param prevLen    Số điểm trong prev
     * @param curr       Mảng LidarPoint của scan N (query)
     * @param currLen    Số điểm trong curr
     * @param initGuess  Ước lượng ban đầu từ odometry delta (thường = {0,0,0})
     * @param result     [OUT] Pose correction (dx, dy, dtheta) để cộng vào robotPose
     * @return true nếu match thành công (đủ correspondences, hội tụ)
     */
    bool match(
        const LidarPoint* prev, int prevLen,
        const LidarPoint* curr, int currLen,
        const Pose2D& initGuess,
        Pose2D& result
    ) {
        // Convert cả 2 scan sang Cartesian (bỏ điểm chất lượng kém)
        int nRef = 0, nQry = 0;
        toCartesian(prev, prevLen, _ref, nRef);
        toCartesian(curr, currLen, _qry, nQry);

        if (nRef < ICP_MIN_PAIRS || nQry < ICP_MIN_PAIRS) {
            return false;   // Không đủ điểm
        }

        // Khởi tạo transform từ initGuess
        float tx    = initGuess.x;
        float ty    = initGuess.y;
        float theta = initGuess.theta;

        // Working copy của query scan (sẽ biến đổi dần)
        ScanPt work[ICP_MAX_PTS];
        memcpy(work, _qry, nQry * sizeof(ScanPt));

        // Apply initGuess lên work
        applyTransform(work, nQry, tx, ty, theta);

        // ── ICP iterations ─────────────────────────────────────
        for (int iter = 0; iter < ICP_MAX_ITER; iter++) {

            // 1. Tìm correspondences (nearest neighbors)
            ScanPt srcPts[ICP_MAX_PTS];  // điểm trong work
            ScanPt dstPts[ICP_MAX_PTS];  // điểm tương ứng trong ref
            int nPairs = 0;

            for (int i = 0; i < nQry; i++) {
                float minDist;
                int   j = nearestNeighbor(work[i], _ref, nRef, minDist);
                if (j < 0 || minDist > ICP_MAX_DIST_M) continue;

                srcPts[nPairs] = work[i];
                dstPts[nPairs] = _ref[j];
                nPairs++;
            }

            if (nPairs < ICP_MIN_PAIRS) break;

            // 2. Solve closed-form 2D rigid transform
            float dTx, dTy, dTheta;
            if (!solveClosed2D(srcPts, dstPts, nPairs, dTx, dTy, dTheta)) break;

            // 3. Apply delta transform lên work
            applyTransform(work, nQry, dTx, dTy, dTheta);

            // 4. Tích luỹ tổng transform
            // Rotate tx,ty bởi dTheta rồi cộng dTx, dTy
            float cosD = cosf(dTheta), sinD = sinf(dTheta);
            float newTx = tx * cosD - ty * sinD + dTx;
            float newTy = tx * sinD + ty * cosD + dTy;
            tx    = newTx;
            ty    = newTy;
            theta = normalizeAngle(theta + dTheta);

            // 5. Kiểm tra hội tụ
            float norm = sqrtf(dTx*dTx + dTy*dTy + dTheta*dTheta);
            if (norm < ICP_EPSILON) break;
        }

        result.x     = tx;
        result.y     = ty;
        result.theta = theta;
        return true;
    }

    /**
     * Tính RMS error giữa 2 scan sau khi đã apply correction.
     * Dùng để log chất lượng ICP.
     */
    float computeRMS(
        const LidarPoint* prev, int prevLen,
        const LidarPoint* curr, int currLen,
        const Pose2D& correction
    ) {
        int nRef = 0, nQry = 0;
        toCartesian(prev, prevLen, _ref, nRef);
        toCartesian(curr, currLen, _qry, nQry);
        if (nRef < 5 || nQry < 5) return -1.0f;

        // Apply correction lên query
        ScanPt work[ICP_MAX_PTS];
        memcpy(work, _qry, nQry * sizeof(ScanPt));
        applyTransform(work, nQry, correction.x, correction.y, correction.theta);

        float sumSq = 0;
        int   count = 0;
        for (int i = 0; i < nQry; i++) {
            float d;
            int j = nearestNeighbor(work[i], _ref, nRef, d);
            if (j >= 0 && d < ICP_MAX_DIST_M) {
                sumSq += d * d;
                count++;
            }
        }
        return (count > 0) ? sqrtf(sumSq / count) : -1.0f;
    }

private:
    // ── Internal scan buffers ──────────────────────────────────
    // Bộ nhớ tĩnh trong SRAM (360 pts × 8 bytes × 2 = 5.6KB)
    // Đủ nhỏ cho SRAM, không cần PSRAM
    ScanPt _ref[ICP_MAX_PTS];
    ScanPt _qry[ICP_MAX_PTS];

    // ── Convert polar LidarPoint → Cartesian ScanPt ──────────
    static void toCartesian(
        const LidarPoint* scan, int len,
        ScanPt* out, int& outLen
    ) {
        outLen = 0;
        for (int i = 0; i < len && outLen < ICP_MAX_PTS; i++) {
            if (!scan[i].quality) continue;
            if (scan[i].distance <= 0.05f || scan[i].distance >= LIDAR_MAX_RANGE) continue;

            // Góc theo radian, KHÔNG cộng robot_heading
            // (ICP so sánh 2 scan trong cùng 1 robot frame)
            float a = scan[i].angle * (M_PI / 180.0f);
            out[outLen].x = scan[i].distance * cosf(a);
            out[outLen].y = scan[i].distance * sinf(a);
            outLen++;
        }
    }

    // ── Apply 2D rigid transform lên mảng điểm ───────────────
    static void applyTransform(
        ScanPt* pts, int n,
        float tx, float ty, float theta
    ) {
        float cosT = cosf(theta), sinT = sinf(theta);
        for (int i = 0; i < n; i++) {
            float x = pts[i].x * cosT - pts[i].y * sinT + tx;
            float y = pts[i].x * sinT + pts[i].y * cosT + ty;
            pts[i].x = x;
            pts[i].y = y;
        }
    }

    // ── Brute-force nearest neighbor ──────────────────────────
    // Returns index trong ref[], hoặc -1 nếu không tìm được
    static int nearestNeighbor(
        const ScanPt& q,
        const ScanPt* ref, int refLen,
        float& outDist
    ) {
        int   best  = -1;
        float bestD = ICP_MAX_DIST_M * ICP_MAX_DIST_M + 1.0f;

        for (int i = 0; i < refLen; i++) {
            float dx = ref[i].x - q.x;
            float dy = ref[i].y - q.y;
            float d2 = dx*dx + dy*dy;
            if (d2 < bestD) {
                bestD = d2;
                best  = i;
            }
        }
        outDist = (best >= 0) ? sqrtf(bestD) : 1e9f;
        return best;
    }

    /**
     * Closed-form 2D rigid transform từ point correspondences.
     *
     * Dựa trên: Arun et al. 1987, phiên bản 2D không cần SVD:
     *   H = Σ (src_i - μ_src)(dst_i - μ_dst)^T   → 2x2 matrix = 4 floats
     *   theta = atan2(H01 - H10, H00 + H11)
     *   t     = μ_dst - R * μ_src
     *
     * @return false nếu giải không hợp lệ (degenerate case)
     */
    static bool solveClosed2D(
        const ScanPt* src, const ScanPt* dst, int n,
        float& tx, float& ty, float& theta
    ) {
        if (n < ICP_MIN_PAIRS) return false;

        // 1. Tính centroid
        float msx = 0, msy = 0, mdx = 0, mdy = 0;
        for (int i = 0; i < n; i++) {
            msx += src[i].x; msy += src[i].y;
            mdx += dst[i].x; mdy += dst[i].y;
        }
        float invN = 1.0f / n;
        msx *= invN; msy *= invN;
        mdx *= invN; mdy *= invN;

        // 2. Cross-covariance H (2x2, dạng tuyến tính)
        float H00 = 0, H01 = 0, H10 = 0, H11 = 0;
        for (int i = 0; i < n; i++) {
            float sx = src[i].x - msx;
            float sy = src[i].y - msy;
            float dx = dst[i].x - mdx;
            float dy = dst[i].y - mdy;
            H00 += sx * dx;
            H01 += sx * dy;
            H10 += sy * dx;
            H11 += sy * dy;
        }

        // 3. Closed-form rotation angle
        theta = atan2f(H01 - H10, H00 + H11);

        // 4. Translation
        float cosT = cosf(theta), sinT = sinf(theta);
        tx = mdx - (msx * cosT - msy * sinT);
        ty = mdy - (msx * sinT + msy * cosT);

        // Sanity: tránh phi thực (NaN)
        if (!isfinite(theta) || !isfinite(tx) || !isfinite(ty)) return false;

        return true;
    }

    // ── Normalize angle to [-π, π] ────────────────────────────
    static inline float normalizeAngle(float a) {
        while (a >  M_PI) a -= 2.0f * M_PI;
        while (a < -M_PI) a += 2.0f * M_PI;
        return a;
    }
};

#endif // ICP_MATCHER_H
