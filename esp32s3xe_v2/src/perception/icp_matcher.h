/**
 * AMR 2.0 — ICP Scan Matcher (v2)
 * Iterative Closest Point (Point-to-Point) with closed-form 2D solution.
 * Header-only, zero malloc, static buffers.
 */

#ifndef ICP_MATCHER_H
#define ICP_MATCHER_H

#include <Arduino.h>
#include <cmath>
#include <cstring>
#include "occupancy_grid.h"  // LidarPoint struct

// ── Config ───────────────────────────────────────────────────
#define ICP_MAX_PTS      360
#define ICP_MAX_ITER     15       // was 10 — more iterations for convergence
#define ICP_EPSILON      1e-4f
#define ICP_MAX_DIST_M   0.25f   // was 0.4 — stricter matching, fewer outliers
#define ICP_MIN_PAIRS    15      // was 20 — allow sparser scans

class IcpMatcher {
public:
    struct Pose2D { float x, y, theta; };
    struct ScanPt { float x, y; };

    bool match(
        const LidarPoint* prev, int prevLen,
        const LidarPoint* curr, int currLen,
        const Pose2D& initGuess, Pose2D& result)
    {
        int nRef = 0, nQry = 0;
        toCartesian(prev, prevLen, _ref, nRef);
        toCartesian(curr, currLen, _qry, nQry);
        if (nRef < ICP_MIN_PAIRS || nQry < ICP_MIN_PAIRS) return false;

        float tx = initGuess.x, ty = initGuess.y, theta = initGuess.theta;

        ScanPt work[ICP_MAX_PTS];
        memcpy(work, _qry, nQry * sizeof(ScanPt));
        applyTransform(work, nQry, tx, ty, theta);

        for (int iter = 0; iter < ICP_MAX_ITER; iter++) {
            ScanPt srcPts[ICP_MAX_PTS], dstPts[ICP_MAX_PTS];
            int nPairs = 0;

            for (int i = 0; i < nQry; i++) {
                float minDist;
                int j = nearestNeighbor(work[i], _ref, nRef, minDist);
                if (j < 0 || minDist > ICP_MAX_DIST_M) continue;
                srcPts[nPairs] = work[i];
                dstPts[nPairs] = _ref[j];
                nPairs++;
            }
            if (nPairs < ICP_MIN_PAIRS) break;

            float dTx, dTy, dTheta;
            if (!solveClosed2D(srcPts, dstPts, nPairs, dTx, dTy, dTheta)) break;

            applyTransform(work, nQry, dTx, dTy, dTheta);

            float cosD = cosf(dTheta), sinD = sinf(dTheta);
            float newTx = tx * cosD - ty * sinD + dTx;
            float newTy = tx * sinD + ty * cosD + dTy;
            tx = newTx; ty = newTy;
            theta = normalizeAngle(theta + dTheta);

            if (sqrtf(dTx*dTx + dTy*dTy + dTheta*dTheta) < ICP_EPSILON) break;
        }

        result.x = tx; result.y = ty; result.theta = theta;
        return true;
    }

    float computeRMS(
        const LidarPoint* prev, int prevLen,
        const LidarPoint* curr, int currLen,
        const Pose2D& correction)
    {
        int nRef = 0, nQry = 0;
        toCartesian(prev, prevLen, _ref, nRef);
        toCartesian(curr, currLen, _qry, nQry);
        if (nRef < 5 || nQry < 5) return -1.0f;

        ScanPt work[ICP_MAX_PTS];
        memcpy(work, _qry, nQry * sizeof(ScanPt));
        applyTransform(work, nQry, correction.x, correction.y, correction.theta);

        float sumSq = 0; int count = 0;
        for (int i = 0; i < nQry; i++) {
            float d;
            int j = nearestNeighbor(work[i], _ref, nRef, d);
            if (j >= 0 && d < ICP_MAX_DIST_M) { sumSq += d * d; count++; }
        }
        return (count > 0) ? sqrtf(sumSq / count) : -1.0f;
    }

private:
    ScanPt _ref[ICP_MAX_PTS], _qry[ICP_MAX_PTS];

    static void toCartesian(const LidarPoint* scan, int len, ScanPt* out, int& outLen) {
        outLen = 0;
        for (int i = 0; i < len && outLen < ICP_MAX_PTS; i++) {
            if (!scan[i].quality || scan[i].distance <= 0.05f || scan[i].distance >= LIDAR_MAX_RANGE) continue;
            float a = scan[i].angle * (M_PI / 180.0f);
            out[outLen].x = scan[i].distance * cosf(a);
            out[outLen].y = scan[i].distance * sinf(a);
            outLen++;
        }
    }

    static void applyTransform(ScanPt* pts, int n, float tx, float ty, float theta) {
        float cosT = cosf(theta), sinT = sinf(theta);
        for (int i = 0; i < n; i++) {
            float x = pts[i].x * cosT - pts[i].y * sinT + tx;
            float y = pts[i].x * sinT + pts[i].y * cosT + ty;
            pts[i].x = x; pts[i].y = y;
        }
    }

    static int nearestNeighbor(const ScanPt& q, const ScanPt* ref, int refLen, float& outDist) {
        int best = -1; float bestD = ICP_MAX_DIST_M * ICP_MAX_DIST_M + 1.0f;
        for (int i = 0; i < refLen; i++) {
            float dx = ref[i].x - q.x, dy = ref[i].y - q.y;
            float d2 = dx*dx + dy*dy;
            if (d2 < bestD) { bestD = d2; best = i; }
        }
        outDist = (best >= 0) ? sqrtf(bestD) : 1e9f;
        return best;
    }

    static bool solveClosed2D(const ScanPt* src, const ScanPt* dst, int n,
                              float& tx, float& ty, float& theta) {
        if (n < ICP_MIN_PAIRS) return false;

        float msx=0,msy=0,mdx=0,mdy=0;
        for (int i=0;i<n;i++) { msx+=src[i].x; msy+=src[i].y; mdx+=dst[i].x; mdy+=dst[i].y; }
        float invN = 1.0f / n;
        msx*=invN; msy*=invN; mdx*=invN; mdy*=invN;

        float H00=0,H01=0,H10=0,H11=0;
        for (int i=0;i<n;i++) {
            float sx=src[i].x-msx, sy=src[i].y-msy;
            float dx=dst[i].x-mdx, dy=dst[i].y-mdy;
            H00+=sx*dx; H01+=sx*dy; H10+=sy*dx; H11+=sy*dy;
        }

        theta = atan2f(H01-H10, H00+H11);
        float cosT=cosf(theta), sinT=sinf(theta);
        tx = mdx - (msx*cosT - msy*sinT);
        ty = mdy - (msx*sinT + msy*cosT);

        return isfinite(theta) && isfinite(tx) && isfinite(ty);
    }

    static inline float normalizeAngle(float a) {
        while (a > M_PI) a -= 2*M_PI;
        while (a < -M_PI) a += 2*M_PI;
        return a;
    }
};

#endif // ICP_MATCHER_H
