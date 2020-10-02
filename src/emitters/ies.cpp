/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <mitsuba/render/emitter.h>
#include <mitsuba/render/medium.h>
#include <mitsuba/core/track.h>
#include <mitsuba/hw/gpuprogram.h>
#include <mitsuba/core/warp.h>
#include <mitsuba/core/plugin.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/fstream.h>
#include <mitsuba/render/mipmap.h>
// #include <mitsuba/hw/gpuprogram.h>
#include <mitsuba/hw/gputexture.h>
// #include <mitsuba/hw/basicshader.h>

MTS_NAMESPACE_BEGIN

#if SPECTRUM_SAMPLES == 3
# define IES_PIXELFORMAT Bitmap::ERGB
#else
# define IES_PIXELFORMAT Bitmap::ESpectrum
#endif

/*!\plugin{point}{Point light source}
 * \icon{emitter_point}
 * \order{1}
 * \parameters{
 *     \parameter{toWorld}{\Transform\Or\Animation}{
 *        Specifies an optional sensor-to-world transformation.
 *        \default{none (i.e. sensor space $=$ world space)}
 *     }
 *     \parameter{position}{\Point}{
 *        Alternative parameter for specifying the light source
 *        position. Note that only one of the parameters
 *        \code{toWorld} and \code{position} can be used at a time.
 *     }
 *     \parameter{intensity}{\Spectrum}{
 *         Specifies the radiant intensity in units of
 *         power per unit steradian.
 *         \default{1}
 *     }
 *     \parameter{samplingWeight}{\Float}{
 *         Specifies the relative amount of samples
 *         allocated to this emitter. \default{1}
 *     }
 * }
 *
 * This emitter plugin implements a simple point light source, which
 * uniformly radiates illumination into all directions.
 */

class IESEmitter : public Emitter {
public:
    typedef TSpectrum<half, SPECTRUM_SAMPLES> SpectrumHalf;
    typedef TMIPMap<Spectrum, SpectrumHalf> MIPMap;
    IESEmitter(const Properties &props) : Emitter(props) {
        m_type |= EDeltaPosition;

        // Log(EInfo, props.toString().c_str());

        if (props.hasProperty("position")) {
            if (props.hasProperty("toWorld"))
                Log(EError, "Only one of the parameters 'position'"
                    " and 'toWorld' can be used!'");
            m_worldTransform = new AnimatedTransform(
                Transform::translate(Vector(props.getPoint("position"))));
        }

        m_intensity = props.getSpectrum("intensity", Spectrum::getD65());
        
        load_ies(props.getString("filename"));

        Log(EInfo, "Able to load texture!");

    }

    void load_ies(std::string fname) {
        fs::path m_filename = Thread::getThread()->getFileResolver()->resolve(fname);

        Log(EInfo, "IES \"%s\"", m_filename.filename().string().c_str());
        if (!fs::exists(m_filename))
            Log(EError, "IES map file \"%s\" could not be found!", m_filename.string().c_str());

        ref<FileStream> fs = new FileStream(m_filename, FileStream::EReadOnly);
        ref<Bitmap> bitmap = new Bitmap(Bitmap::EAuto, fs);

        Properties rfilterProps("lanczos");
        rfilterProps.setInteger("lobes", 2);
        ref<ReconstructionFilter> rfilter = static_cast<ReconstructionFilter *> (
            PluginManager::getInstance()->createObject(
            MTS_CLASS(ReconstructionFilter), rfilterProps));
        rfilter->configure();


        m_mipmap = new MIPMap(bitmap, IES_PIXELFORMAT, Bitmap::EFloat, rfilter,
            ReconstructionFilter::EClamp, ReconstructionFilter::EClamp, EEWA, 10.0f,
            fs::path(), 0, std::numeric_limits<Float>::infinity(), Spectrum::EIlluminant);


        // Point2 uv(0.f,0.f);
        // // if(uv[0]<0.f) uv[0] = 0.5f-uv[0];
        // Log(EInfo, "evaldir u:%f, v:%f, val:%s", uv[0], uv[1], m_mipmap->evalTexel(0,0,0).toString().c_str());
        
        // // Log(EInfo, "evaldir u:%f, v:%f, val:%s", uv[0], uv[1], m_mipmap->evalBox(0,uv).toString().c_str());
        
        // uv[0] = 1.f; uv[1] = 0.f;
        // Log(EInfo, "evaldir u:%f, v:%f, val:%s", uv[0], uv[1], m_mipmap->evalTexel(0,255,0).toString().c_str());
        
        // // Log(EInfo, "evaldir u:%f, v:%f, val:%s", uv[0], uv[1], m_mipmap->evalBox(0,uv).toString().c_str());
        
        // uv[0] = 0.f; uv[1] = 1.0f;
        // Log(EInfo, "evaldir u:%f, v:%f, val:%s", uv[0], uv[1], m_mipmap->evalBox(0,uv).toString().c_str());
        

        // uv[0] = 1.0f; uv[1] = 1.0f;
        // Log(EInfo, "evaldir u:%f, v:%f, val:%s", uv[0], uv[1], m_mipmap->evalBox(0,uv).toString().c_str());
        
    }


    IESEmitter(Stream *stream, InstanceManager *manager)
     : Emitter(stream, manager) {
        configure();
        m_intensity = Spectrum(stream);
        std::string fname = stream->readString();
        load_ies(fname);
    }

    void serialize(Stream *stream, InstanceManager *manager) const {
        Emitter::serialize(stream, manager);
        stream->writeString(m_filename.string());
        m_intensity.serialize(stream);
        
    }

    Spectrum samplePosition(PositionSamplingRecord &pRec, const Point2 &sample,
            const Point2 *extra) const {
        const Transform &trafo = m_worldTransform->eval(pRec.time);
        pRec.p = trafo(Point(0.0f));
        pRec.n = Normal(0.0f);
        pRec.pdf = 1.0f;
        pRec.measure = EDiscrete;
        return m_intensity * (4 * M_PI);
    }

    Spectrum evalPosition(const PositionSamplingRecord &pRec) const {
        return (pRec.measure == EDiscrete) ? (m_intensity * 4*M_PI) : Spectrum(0.0f);
    }

    Float pdfPosition(const PositionSamplingRecord &pRec) const {
        return (pRec.measure == EDiscrete) ? 1.0f : 0.0f;
    }

    Spectrum sampleDirection(DirectionSamplingRecord &dRec,
            PositionSamplingRecord &pRec,
            const Point2 &sample,
            const Point2 *extra) const {
        const Transform &trafo = m_worldTransform->eval(pRec.time);
        
        Vector d = warp::squareToUniformSphere(sample);
        dRec.d = trafo(d);
        // dRec.d = warp::squareToUniformSphere(sample);
        dRec.pdf = INV_FOURPI;
        dRec.measure = ESolidAngle;
        return evalDirection(dRec, pRec) / dRec.pdf;
    }

    Float pdfDirection(const DirectionSamplingRecord &dRec,
            const PositionSamplingRecord &pRec) const {
        return (dRec.measure == ESolidAngle) ? INV_FOURPI : 0.0f;
    }

    Spectrum evalDirection(const DirectionSamplingRecord &dRec,
            const PositionSamplingRecord &pRec) const {

        const Transform &trafo = m_worldTransform->eval(pRec.time);
        Vector local = trafo.inverse()(dRec.d);

        // Point2 uv(
        //     std::atan2(local.x, -local.z) * INV_TWOPI,
        //     math::safe_acos(local.y) * INV_PI
        // );

        // Point2 uv(
        //     std::atan2(local.y, local.x) * INV_TWOPI,
        //     math::safe_acos(local.z) * INV_PI
        // );

        Point2 uv(
            math::safe_acos(local.z) * INV_PI,
            std::atan2(local.y, local.x) * INV_TWOPI
            
        );

        if(uv[0]<0.f) uv[0] = 1.f+uv[0];


        // Log(EInfo, "evaldir u:%f, v:%f, val:%s", uv[0], uv[1], m_mipmap->evalBox(0,uv).toString().c_str());
        return Spectrum((dRec.measure == ESolidAngle) ? INV_FOURPI*m_mipmap->evalBox(0, uv) : Spectrum(0.0f));
        // return Spectrum((dRec.measure == ESolidAngle) ? INV_FOURPI : (0.0f));
    }

    Spectrum sampleRay(Ray &ray,
            const Point2 &spatialSample,
            const Point2 &directionalSample,
            Float time) const {
        const Transform &trafo = m_worldTransform->eval(time);
        Vector local = warp::squareToUniformSphere(directionalSample);
        ray.setTime(time);
        ray.setOrigin(trafo(Point(0.0f)));
        ray.setDirection(trafo(local));
        
        
        // Point2 uv(
        //     std::atan2(local.x, -local.z) * INV_TWOPI,
        //     math::safe_acos(local.y) * INV_PI
        // );
        
        // Point2 uv(
        //     std::atan2(local.y, local.x) * INV_TWOPI,
        //     math::safe_acos(local.z) * INV_PI
        // );
        
        Point2 uv(
            math::safe_acos(local.z) * INV_PI,
            std::atan2(local.y, local.x) * INV_TWOPI
            
        );
        if(uv[0]<0.f) uv[0] = 1.f+uv[0];
        
        // Log(EInfo, "sampleRay u:%f, v:%f, val:%s", uv[0], uv[1], m_mipmap->evalBox(0,uv).toString().c_str());
        
        return m_intensity * (4*M_PI) * m_mipmap->evalBox(0,uv);
        // return m_intensity * (4*M_PI);
        
    }

    Spectrum sampleDirect(DirectSamplingRecord &dRec, const Point2 &sample) const {
        const Transform &trafo = m_worldTransform->eval(dRec.time);

        dRec.p = trafo.transformAffine(Point(0.0f));
        dRec.pdf = 1.0f;
        dRec.measure = EDiscrete;
        dRec.uv = Point2(0.5f);
        dRec.d = dRec.p - dRec.ref;
        dRec.dist = dRec.d.length();
        Float invDist = 1.0f / dRec.dist;
        dRec.d *= invDist;
        dRec.n = Normal(0.0f);
        dRec.pdf = 1;
        dRec.measure = EDiscrete;

        Vector local = trafo.inverse()(-dRec.d);
     
        // Point2 uv(
        //     std::atan2(local.x, -local.z) * INV_TWOPI,
        //     math::safe_acos(local.y) * INV_PI
        // );

        // Point2 uv(
        //     std::atan2(local.y, local.x) * INV_TWOPI,
        //     math::safe_acos(local.z) * INV_PI
        // );

        Point2 uv(
            math::safe_acos(local.z) * INV_PI,
            std::atan2(local.y, local.x) * INV_TWOPI
            
        );

        if(uv[0]<0.f) uv[0] = 1.f+uv[0];
        

        // Log(EInfo, "sampleDirect u:%f, v:%f, val:%s", uv[0], uv[1], m_mipmap->evalBox(0,uv).toString().c_str());
        
        return m_intensity * (invDist * invDist) * m_mipmap->evalBox(0,uv);
        // return m_intensity * (invDist * invDist);
    }

    Float pdfDirect(const DirectSamplingRecord &dRec) const {
        return dRec.measure == EDiscrete ? 1.0f : 0.0f;
    }

    // void addChild(const std::string &name, ConfigurableObject *child) {
    //     if (child->getClass()->derivesFrom(MTS_CLASS(Texture)) && name == "texture") {
    //         m_texture = static_cast<Texture *>(child);
    //     } else {
    //         Emitter::addChild(name, child);
    //     }
    // }

    AABB getAABB() const {
        return m_worldTransform->getTranslationBounds();
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "IESEmitter[" << endl
            << "  worldTransform = " << indent(m_worldTransform.toString()) << "," << endl
            << "  intensity = " << m_intensity.toString() << "," << endl
            // << "  texture = " << m_texture.toString() << "," << std::endl
            << "  medium = " << indent(m_medium.toString())
            << "]";
        return oss.str();
    }

    Shader *createShader(Renderer *renderer) const;

    MTS_DECLARE_CLASS()
private:
    Spectrum m_intensity;
    MIPMap *m_mipmap;
    fs::path m_filename;
    
    // ref<const Texture> m_texture;
};


// ================ Hardware shader implementation ================

class IESEmitterShader : public Shader {
public:
    IESEmitterShader(Renderer *renderer, const Spectrum &intensity,
        const fs::path &filename)
        : Shader(renderer, EEmitterShader), m_intensity(intensity), m_filename(filename) {
    }

    void resolve(const GPUProgram *program, const std::string &evalName,
            std::vector<int> &parameterIDs) const {
        parameterIDs.push_back(program->getParameterID(evalName + "_intensity", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_mipmap", false));
    }

    void generateCode(std::ostringstream &oss, const std::string &evalName,
            const std::vector<std::string> &depNames) const {
        oss << "uniform mat4 " << evalName << "_worldToEmitter;" << endl
            << "vec3 " << evalName << "_dir(vec3 wo) {" << endl
            << "    vec3 local = (" << evalName << "_worldToEmitter * vec4(wo, 0)).xyz;" << endl
            // << "    intersection its;" << endl
            // << "    its.hasUVPartials = false;" << endl
            // << "    its.uv = vec2(std::atan(local.x, -local.z) * INV_TWOPI, acos(local.y) * INV_PI);" << endl 
            // << "    return " << depNames[0] << "(its) * inv_fourpi;" << endl
            << "    return " << "vec3(inv_fourpi);" << endl
            << "}" << endl;
    }

    void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
        int &textureUnitOffset) const {
        program->setParameter(parameterIDs[0], m_intensity);
    }

    MTS_DECLARE_CLASS()
private:
    Spectrum m_intensity;
    MIPMap *m_mipmap;
    fs::path m_filename;
};

Shader *IESEmitter::createShader(Renderer *renderer) const {
    return new IESEmitterShader(renderer, m_intensity, m_filename);
}

MTS_IMPLEMENT_CLASS(IESEmitterShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(IESEmitter, false, Emitter)
MTS_EXPORT_PLUGIN(IESEmitter, "IES emitter");
MTS_NAMESPACE_END
