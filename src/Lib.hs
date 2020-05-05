{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE TypeApplications, DataKinds, GADTs #-}
{-# LANGUAGE FlexibleInstances, MultiParamTypeClasses, FunctionalDependencies #-}


module Lib
    ( someFunc
    ) where


import Data.Maybe ( fromMaybe )
import Data.Int (Int32)
import Linear.V3
import Linear.V4
import Linear.Matrix
import Linear.Metric (dot, norm)
import Graphics.UI.GLUT ( Key(..), KeyState(..) )
import Vis
import Vis.Vis ( vis )
import Vis.Camera ( makeCamera, setCamera, cameraMotion )
import qualified Vis.GlossColor as GlossColor
import Control.Lens
--import Debug.Trace
import Control.Concurrent.Async (Async, async, cancel)
import Data.IORef
import OpenCV.HighGui
import OpenCV.Core.Types.Mat
import Data.Function (on)
import Control.Applicative
import Control.Monad (forever)

mkSlider :: [IORef Double] -> IO (Async ())
mkSlider refs = async $ do
    win <- makeWindow "controller"
    resizeWindow win 300 300
    createTrackbar win "1" 360 (2*360) $ \v -> writeIORef (refs !! 0) (fromIntegral v / (180.0 / pi) )
    createTrackbar win "2" 360 (2*360) $ \v -> writeIORef (refs !! 1) (fromIntegral v / (180.0 / pi) )
    createTrackbar win "3" 360 (2*360) $ \v -> writeIORef (refs !! 2) (fromIntegral v / (180.0 / pi) )
    createTrackbar win "4" 360 (2*360) $ \v -> writeIORef (refs !! 3) (fromIntegral v / (180.0 / pi) )
    createTrackbar win "5" 360 (2*360) $ \v -> writeIORef (refs !! 4) (fromIntegral v / (180.0 / pi) )
    createTrackbar win "6" 360 (2*360) $ \v -> writeIORef (refs !! 5) (fromIntegral v / (180.0 / pi) )
    imshow win $ eyeMat @Int32 @Int32  @Int32 100 300 3 Depth_8U
    forever $ waitKey 0


rotM33XYZ :: (Floating f) => f -> f -> f -> M33 f
rotM33XYZ r p y = rotM33Z y !*! rotM33Y p !*! rotM33X r

rotM33X :: (Floating f) => f -> M33 f
rotM33X a = V3 (V3 1 0 0) (V3 0 ca (-sa))  (V3 0 sa ca) where
    ca = cos a
    sa = sin a

rotM33Y :: (Floating f) => f -> M33 f
rotM33Y a = V3 (V3 ca 0 sa) (V3 0 1 0)  (V3 (-sa) 0 ca) where
    ca = cos a
    sa = sin a

rotM33Z :: (Floating f) => f -> M33 f
rotM33Z a = V3 (V3 ca (-sa) 0)  (V3 sa ca 0) (V3 0 0 1) where
    ca = cos a
    sa = sin a

printV4 :: (Show f) => V4 f -> IO ()
printV4 (V4 v1 v2 v3 v4) = print [v1, v2, v3, v4]

printM44 :: (Show f) => M44 f -> IO ()
printM44 (V4 v1 v2 v3 v4) = do
    putStrLn ""
    printV4 v1
    printV4 v2
    printV4 v3
    printV4 v4
    putStrLn ""

rotM44X :: (Floating f) => f -> M44 f
rotM44X a = identity & _m33 .~ rotM33X a

rotM44Z :: (Floating f) => f -> M44 f
rotM44Z a = identity & _m33 .~ rotM33Z a

transM44X :: (Floating f) => f -> M44 f
transM44X a =  identity & translation .~ V3 a 0 0

transM44Z :: (Floating f) => f -> M44 f
transM44Z a =  identity & translation .~ V3 0 0 a

twistX :: (Floating f) => f -> f -> M44 f
twistX d theta = transM44X d !*! rotM44X theta

twistZ :: (Floating f) => f -> f -> M44 f
twistZ d theta = transM44Z d !*! rotM44Z theta

class Renderable b f  | b -> f where
    renderFC :: Flavour -> Color -> b -> VisObject f
    render   :: b -> VisObject f
    render = renderFC Wireframe blue

-- M44 を render
instance (Floating f) => Renderable (M44 f) f where
    renderFC _ _ m = convfrm m $ axes (0.05, 10)
        where
        axes (size, aspectRatio) = VisObjects [ Arrow (size, aspectRatio) (V3 1 0 0) (GlossColor.makeColor 1 0 0 1)
                                              , Arrow (size, aspectRatio) (V3 0 0 1) (GlossColor.makeColor 0 0 1 1)
                                              ]

class FrameConvertable b f | b -> f where
    convfrm :: M44 f -> b -> b

instance (Floating f) => FrameConvertable (VisObject f) f where
    convfrm m v = Trans t $ RotDcm rot v
        where
        rot = transpose $ m ^. _m33 -- NOTE: vis function use row vector??
        t   = m ^. translation

---- for rendering and collision detection
data LinkShape f = OBB (V3 f) (V3 f) (M33 f) -- center size/2 axis-vector
    deriving Show

instance (Floating f) => FrameConvertable (LinkShape f) f where
    convfrm m (OBB c a r) = OBB c' a r'
        where
        c' = (m !* point c) ^. _xyz
        r' = m^._m33 !*! r

instance (Floating f) => Renderable (LinkShape f) f where
    renderFC f color (OBB c b@(V3 bx by bz) m) = Trans c $ RotDcm rot $ Box (2*bx, 2*by, 2*bz) f color
        where
        rot = transpose $ m ^. _m33 -- NOTE: vis function use row vector??

hasIntersection :: (Ord f, Floating f) => LinkShape f -> LinkShape f -> Bool
hasIntersection (OBB c0 a@(V3 a0 a1 a2)  m0) (OBB c1 b@(V3 b0 b1 b2) m1) = not $ easy || i0 || i1 || i2 || i3 || i4 || i5 || i6 || i7 || i8 || i9 || i10 || i11 || i12 || i13 || i14
    where

    -- not intersection test using bounding sphere
    easy = norm a + norm b < norm vD

    -- not intersection test
    i0  = a0 + (b0*c00' + b1*c01' + b2*c02') < abs sA0D
    i1  = a1 + (b0*c10' + b1*c11' + b2*c12') < abs sA1D
    i2  = a2 + (b0*c20' + b1*c21' + b2*c22') < abs sA2D

    i3  = (a0*c00' + a1*c10' + a2*c20') + b0 < abs (vB0 `dot` vD)
    i4  = (a0*c01' + a1*c11' + a2*c21') + b1 < abs (vB1 `dot` vD)
    i5  = (a0*c02' + a1*c12' + a2*c22') + b2 < abs (vB2 `dot` vD)

    i6  = (a1*c20' + a2*c10') + (b1*c02' + b2*c01') < abs (c10*sA2D - c20*sA1D)
    i7  = (a1*c21' + a2*c11') + (b0*c02' + b2*c00') < abs (c11*sA2D - c21*sA1D)
    i8  = (a1*c22' + a2*c12') + (b0*c01' + b1*c00') < abs (c12*sA2D - c22*sA1D)

    i9  = (a0*c20' + a2*c00') + (b1*c12' + b2*c11') < abs (c20*sA0D - c00*sA2D)
    i10 = (a0*c21' + a2*c01') + (b0*c12' + b2*c10') < abs (c21*sA0D - c01*sA2D)
    i11 = (a0*c22' + a2*c02') + (b0*c11' + b1*c10') < abs (c22*sA0D - c02*sA2D)

    i12 = (a0*c10' + a1*c00') + (b1*c22' + b2*c21') < abs (c00*sA1D - c10*sA0D)
    i13 = (a0*c11' + a1*c01') + (b0*c22' + b2*c20') < abs (c01*sA1D - c11*sA0D)
    i14 = (a0*c12' + a1*c02') + (b0*c21' + b1*c20') < abs (c02*sA1D - c12*sA0D)

    vD = c1 - c0
    V3 vA0 vA1  vA2 = transpose m0
    V3 vB0 vB1  vB2 = transpose m1
    V3 (V3 c00 c01 c02) (V3 c10 c11 c12) (V3 c20 c21 c22)  = transpose m0 !*! m1

    ((c00', c01', c02'), (c10', c11', c12'), (c20', c21', c22'))  = ((abs c00, abs c01, abs c02), (abs c10, abs c11, abs c12), (abs c20, abs c21, abs c22))

    sA0D = vA0 `dot` vD
    sA1D = vA1 `dot` vD
    sA2D = vA2 `dot` vD

-- TODO
hasCollisions :: (Ord f, Floating f) => [LinkShape f] -> [Bool]
hasCollisions (x:xs) = any id (hasIntersection x <$> xs) : hasCollisions xs
hasCollisions [] = []

aabb :: (Ord f, Floating f) => (f, f) -> (f, f) -> (f, f) -> LinkShape f
aabb (x0,x1) (y0,y1) (z0,z1) = OBB c b identity
        where
        c = V3 ((x1+x0)/2) ((y1+y0)/2) ((z1+z0)/2)
        b = V3 ((x1-x0)/2) ((y1-y0)/2) ((z1-z0)/2)

type FrameTrans a = a -> M44 a

data DH a = DH {   __a :: a      -- [m]
               ,   __d :: a      -- [m]
               ,   __alpha :: a  -- [rad]
               } deriving Show

makeLenses ''DH

d1 = 0.089159
a2 = 0.425
a3 = 0.39225
d4 = 0.10915
d5 = 0.09465
d6 = 0.0823

ur5dh :: [DH Double]
ur5dh =
    [ DH 0             d1          (pi/2) -- J1
    , DH (-a2)         0           0      -- J2
    , DH (-a3)         0           0      -- J3
    , DH 0             d4          (pi/2) -- J4
    , DH 0             d5          (-pi/2)-- J5
    , DH 0             d6          0      -- J6(flange)
    ]

-- how to draw link (from frame{i} to frame{i+1})
ur5link :: [LinkShape Double]
ur5link =
    [ aabb (-r/2, r/2) (-d1, r/2) (-r/2, r/2) -- 1,2
    , aabb (-r/2, a2 + r/2) (-r/2, r/2) (delta+r/2, 3*r/2) -- 2,3
    , aabb (-r/2, a3 + r/2) (-r/2, r/2) (-r/2, r/2) -- 3,4
    , aabb (-r/2, r/2) (r/2 + delta-d4, r/2) (-r/2, r/2) -- 4,5
    , aabb (-r/2, r/2) (-r/2, d5 - r/2 - delta) (-r/2, r/2) -- 5,6
    , aabb (-r/2, r/2) (-r/2, r/2) (delta-d6+r/2, 0) -- 6,frange
    , aabb (-r/2, r/2) (-r/2, r/2) (delta, 3*r)  -- tool
    ]
    where
    r = 0.05
    delta = 0.01

-- https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
dr2ft :: (Floating f) => DH f -> FrameTrans f
dr2ft dh theta = twistZ (dh ^. _d) theta !*!  twistX (dh^. _a) (dh^. _alpha)

ur5fts :: [FrameTrans Double]
ur5fts = dr2ft <$> ur5dh

-- calc each frame in base-coordinate for each joint-angle
kinematics :: (Floating f) => [FrameTrans f] -> [f] -> [M44 f]
kinematics (e:es) (a:as) = t : ((t !*! ) <$> kinematics es as)
    where
    t = e a
kinematics [] [] = []
kinematics _ _ = error "invalid args"

newtype World = World Double

renderUR :: [IORef Double] -> Float -> IO (VisObject Double)
renderUR refs _ = do
    jangles <- mapM readIORef refs
    let frames =  identity : kinematics ur5fts jangles
    let vframes = render <$> frames
    let ts     = tail frames ++ [last frames]
    let links  = zipWith convfrm ts ur5link
    let colors  = (\b -> if b then red else blue) <$> hasCollisions links
    let vlinks  = zipWith (renderFC Wireframe) colors links
    return $ VisObjects $ vlinks ++ vframes

test1 refs _ = do
    xs <- mapM readIORef refs
    let rot = rotM33XYZ (xs!!3) (xs!!4) (xs!!5)
    let t   = V3 (xs!!0) (xs!!1) (xs!!2)
    let m = identity & _m33 .~ rot & translation .~ t
    --let obb  = OBB (V3 0.1 0.2 0.3) (V3 0.1 0.2 0.3) identity
    let obb  = OBB (V3 0 0 0.3) (V3 0.1 0.2 0.3) identity
    let vobb0 = renderFC Wireframe green $ convfrm m obb
    let vobb1 = convfrm m $ renderFC Wireframe red obb
    return $ VisObjects [vobb0, vobb1]

test2 refs _ = do
    xs <- mapM readIORef refs
    let obb0 = aabb (-0.10,0.1) (-0.2,0.2) (-0.3,0.3)
    let obb1 = OBB (V3 0 (xs!!1) (xs!!2)) (V3 0.1 0.2 0.3) (rotM33XYZ (xs!!3) (xs!!4) (xs!!5))
    let color = if hasIntersection obb0 obb1 then red else blue
    return $ VisObjects $ render (identity :: M44 Double) : (renderFC Wireframe color <$> [obb0, obb1])


someFunc :: IO ()
someFunc = do
    refs <- mapM (newIORef @Double) [0, 0, 0, 0, 0, 0]
    tid <- mkSlider refs
    --animateIO defaultOpts (test1 refs)
    animateIO defaultOpts (renderUR refs)
    cancel tid
    return ()


main = someFunc


