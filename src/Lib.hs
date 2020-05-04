{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE DataKinds, GADTs #-}
{-# LANGUAGE FlexibleInstances, MultiParamTypeClasses, FunctionalDependencies #-}


module Lib
    ( someFunc
    ) where


import Data.Maybe ( fromMaybe )
import Data.Int (Int32)
import Control.Applicative
import Linear.V3
import Linear.V4
import Linear.Matrix
import Graphics.UI.GLUT ( Key(..), KeyState(..) )
import Vis
import Vis.Vis ( vis )
import Vis.Camera ( makeCamera, setCamera, cameraMotion )
import qualified Vis.GlossColor as GlossColor
import Control.Lens
--import Debug.Trace

rotMat33X :: (Floating f) => f -> M33 f
rotMat33X a = V3 (V3 1 0 0) (V3 0 ca (-sa))  (V3 0 sa ca) where
    ca = cos a
    sa = sin a

rotMat33Z :: (Floating f) => f -> M33 f
rotMat33Z a = V3 (V3 ca (-sa) 0)  (V3 sa ca 0) (V3 0 0 1) where
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
rotM44X a = identity & _m33 .~ rotMat33X a

rotM44Z :: (Floating f) => f -> M44 f
rotM44Z a = identity & _m33 .~ rotMat33Z a

transM44X :: (Floating f) => f -> M44 f
transM44X a =  identity & translation .~ V3 a 0 0

transM44Z :: (Floating f) => f -> M44 f
transM44Z a =  identity & translation .~ V3 0 0 a

twistX :: (Floating f) => f -> f -> M44 f
twistX d theta = transM44X d !*! rotM44X theta

twistZ :: (Floating f) => f -> f -> M44 f
twistZ d theta = transM44Z d !*! rotM44Z theta

---- i-th Link
--class IOpenLinkElm elm where
--    Rt :: (Floating f) => elm -> f -> (M33 f, V3 f) -- calc frame{i+1} with θ_{i+1}
--    m4 :: (Floating f) => elm -> f -> M44

class Renderable b f  | b -> f where
    render :: b -> VisObject f

visM44 :: M44 f -> VisObject f -> VisObject f
visM44 m v = Trans t $ RotDcm rot v
    where
    rot = transpose $ m ^. _m33 -- NOTE: vis function use row vector??
    t   = m ^. translation

-- M44 を render
instance (Floating f) => Renderable (M44 f) f where
    render m = visM44 m $ axes (0.05, 10)
        where
        axes (size, aspectRatio) = VisObjects [ Arrow (size, aspectRatio) (V3 1 0 0) (GlossColor.makeColor 1 0 0 1)
                                              , Arrow (size, aspectRatio) (V3 0 0 1) (GlossColor.makeColor 0 0 1 1)
                                              ]

---- for rendering and collision detection
data LinkShape f where
    AABB :: (Floating f) => (f, f) -> (f, f) -> (f, f) -> LinkShape f

instance (Floating f) => Renderable (LinkShape f) f where
    render (AABB (x0,x1) (y0,y1) (z0,z1)) = Trans t $ Box (x1-x0, y1-y0, z1-z0) Wireframe blue
        where
        t = V3 (x0 + (x1-x0)/2) (y0 + (y1-y0)/2) (z0 + (z1-z0)/2)

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
    [ AABB (-r/2, r/2) (-r/2, r/2) (0, d1+r/2) -- 1,2
    , AABB (-a2-r/2, r/2) (-r/2, r/2) (delta+r/2, 3*r/2) -- 2,3
    , AABB (-a3-r/2, r/2) (-r/2, r/2) (-r/2, r/2) -- 3,4
    , AABB (-r/2, r/2) (-r/2, r/2) (delta+r/2, d4+r/2) -- 4,5
    , AABB (-r/2, r/2) (-r/2, r/2) (delta+r/2, d5 + r/2) -- 5,6
    , AABB (-r/2, r/2) (-r/2, r/2) (delta+r/2, d6)  -- 6,frange
    ]
    where
    r = 0.1
    delta = 0.01

type FrameTrans a = a -> M44 a 

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

renderUR :: Float -> IO (VisObject Double)
--renderUR _ = return $ VisObjects $ render <$> (identity : kinematics ur5fts [0, 0, 0, 0, 0, 0])
renderUR _ = do
    let jangles = [0, 0, 0, 0, 0, 0]
    let frames =  identity : kinematics ur5fts jangles
    --mapM_ printM44 $ zipWith ($) ur5fts jangles
    let vframes = render <$> frames
    let links  = zipWith visM44 frames $ render <$> ur5link
    return (VisObjects $ links ++ vframes)

someFunc :: IO ()
someFunc = animateIO defaultOpts renderUR

main = someFunc
