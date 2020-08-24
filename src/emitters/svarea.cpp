#include <mitsuba/render/emitter.h>
#include <mitsuba/render/shape.h>
#include <mitsuba/render/medium.h>
#include <mitsuba/hw/gpuprogram.h>
#include <mitsuba/core/warp.h>

MTS_NAMESPACE_BEGIN

class SpatialVaryingArea : public Emitter {
public:
    SpatialVaryingArea(const Properties &props) : Emitter(props) {
        m_type |= EOnSurface;

        if (props.hasProperty("toWorld"))
            Log(EError, "Found a 'toWorld' transformation -- this is not "
                "allowed -- the area light inherits this transformation from "
                "its parent shape");

        m_radiance = props.getSpectrum("radiance", Spectrum::getD65());
        m_power = Spectrum(0.0f); /// Don't know the power yet

        m_cutoffAngle = props.getFloat("cutoffAngle", 20);
        m_beamWidth = props.getFloat("beamWidth", m_cutoffAngle * 3.0f/4.0f);
        m_beamWidth = degToRad(m_beamWidth);
        m_cutoffAngle = degToRad(m_cutoffAngle);
        Assert(m_cutoffAngle >= m_beamWidth);
        

    }

    SpatialVaryingArea(Stream *stream, InstanceManager *manager)
        : Emitter(stream, manager) {
        m_radiance = Spectrum(stream);
        m_power = Spectrum(stream);
        m_beamWidth = stream->readFloat();
        m_cutoffAngle = stream->readFloat();
        configure();
    }

    void configure() {
        m_cosBeamWidth = std::cos(m_beamWidth);
        m_cosCutoffAngle = std::cos(m_cutoffAngle);
        // m_uvFactor = std::tan(m_cutoffAngle);
        m_invTransitionWidth = 1.0f / (m_cutoffAngle - m_beamWidth);
    }

    inline Spectrum falloffCurve(const Vector &d) const {
        const Float cosTheta = Frame::cosTheta(d);

        if (cosTheta <= m_cosCutoffAngle)
            return Spectrum(0.0f);

        Spectrum result(1.0f);
        // if (m_texture->getClass() != MTS_CLASS(ConstantSpectrumTexture)) {
        //     Intersection its;
        //     its.hasUVPartials = false;
        //     its.uv = Point2(0.5f + 0.5f * d.x / (d.z * m_uvFactor),
        //                     0.5f + 0.5f * d.y / (d.z * m_uvFactor));
        //     result = m_texture->eval(its);
        // }

        if (cosTheta >= m_cosBeamWidth)
            return result;

        return result * ((m_cutoffAngle - std::acos(cosTheta))
                * m_invTransitionWidth);
    }

    void serialize(Stream *stream, InstanceManager *manager) const {
        Emitter::serialize(stream, manager);
        m_radiance.serialize(stream);
        m_power.serialize(stream);
        stream->writeFloat(m_beamWidth);
        stream->writeFloat(m_cutoffAngle);
    }

    Spectrum samplePosition(PositionSamplingRecord &pRec,
            const Point2 &sample, const Point2 *extra) const {
        m_shape->samplePosition(pRec, sample);
        // figure out where is it used? this function should be modulated based on the outgoing dir
        return m_power;
    }

    // figure out where is it used? this function should be modulated based on the outgoing dir
    Spectrum evalPosition(const PositionSamplingRecord &pRec) const {
        return m_radiance * M_PI;
    }

    Spectrum eval(const Intersection &its, const Vector &d) const {
 
        if (dot(its.shFrame.n, d) <= 0)
            return Spectrum(0.0f);
        else {
        	Vector local = its.geoFrame.toLocal(d);
            return m_radiance * falloffCurve(local);
        }
    }

    Float pdfPosition(const PositionSamplingRecord &pRec) const {
        return m_shape->pdfPosition(pRec);
    }

    Spectrum sampleDirection(DirectionSamplingRecord &dRec,
            PositionSamplingRecord &pRec,
            const Point2 &sample, const Point2 *extra) const {
        Vector local = warp::squareToUniformCone(m_cosCutoffAngle, sample);
        dRec.d = Frame(pRec.n).toWorld(local);
        dRec.pdf = warp::squareToUniformConePdf(m_cosCutoffAngle);
        dRec.measure = ESolidAngle;
        return Spectrum(1.0f);
    }

    Spectrum evalDirection(const DirectionSamplingRecord &dRec,
            const PositionSamplingRecord &pRec) const {
        Float dp = dot(dRec.d, pRec.n);

        if (dRec.measure != ESolidAngle || dp < 0 )
            dp = 0.0f;

        const Transform &trafo = m_worldTransform->eval(pRec.time);
        return Spectrum(falloffCurve(trafo.inverse()(dRec.d)) * INV_PI * dp);
    }

    Float pdfDirection(const DirectionSamplingRecord &dRec,
            const PositionSamplingRecord &pRec) const {
        Float dp = dot(dRec.d, pRec.n);

        if (dRec.measure != ESolidAngle || dp < 0 || dp <= m_cosCutoffAngle)
            dp = 0.0f;

        return INV_PI * dp * warp::squareToUniformConePdf(m_cosCutoffAngle);
    }

    Spectrum sampleRay(Ray &ray,
            const Point2 &spatialSample,
            const Point2 &directionalSample,
            Float time) const {
        PositionSamplingRecord pRec(time);
        m_shape->samplePosition(pRec, spatialSample);
        Vector local = warp::squareToUniformCone(
            					m_cosCutoffAngle, directionalSample);
        ray.setTime(time);
        ray.setOrigin(pRec.p);
        ray.setDirection(Frame(pRec.n).toWorld(local));
        Float dirPdf = warp::squareToUniformConePdf(m_cosCutoffAngle);
        
        return m_power * falloffCurve(local)/ dirPdf;
    }

    Spectrum sampleDirect(DirectSamplingRecord &dRec,
            const Point2 &sample) const {
        m_shape->sampleDirect(dRec, sample);

        /* Check that the emitter and reference position are oriented correctly
           with respect to each other. Note that the >= 0 check
           for 'refN' is intentional -- those sampling requests that specify
           a reference point within a medium or on a transmissive surface
           will set dRec.refN = 0, hence they should always be accepted. */
        if (dot(dRec.d, dRec.refN) >= 0 && dot(dRec.d, dRec.n) < 0 && dRec.pdf != 0) {
        	const Transform &trafo = m_worldTransform->eval(dRec.time);
            return m_radiance * falloffCurve(trafo.inverse()(-dRec.d)) / dRec.pdf;
        } else {
            dRec.pdf = 0.0f;
            return Spectrum(0.0f);
        }
    }

    // check this
    Float pdfDirect(const DirectSamplingRecord &dRec) const {
        /* Check that the emitter and receiver are oriented correctly
           with respect to each other. */
    	Float dp = dot(dRec.d, dRec.n);
        if (dot(dRec.d, dRec.refN) >= 0 && dp < 0 && dp <=m_cosCutoffAngle) {
            return m_shape->pdfDirect(dRec);
        } else {
            return 0.0f;
        }
    }

    void setParent(ConfigurableObject *parent) {
        Emitter::setParent(parent);

        if (parent->getClass()->derivesFrom(MTS_CLASS(Shape))) {
            Shape *shape = static_cast<Shape *>(parent);
            if (m_shape == shape || shape->isCompound())
                return;

            if (m_shape != NULL)
                Log(EError, "An area light cannot be parent of multiple shapes");

            m_shape = shape;
            m_shape->configure();
            m_power = m_radiance * M_PI * m_shape->getSurfaceArea();
        } else {
            Log(EError, "An area light must be child of a shape instance");
        }
    }

    AABB getAABB() const {
        return m_shape->getAABB();
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "SpatialVaryingArea[" << endl
            << "  radiance = " << m_radiance.toString() << "," << endl
            << "  beamWidth = " << (m_beamWidth * 180/M_PI) << "," << std::endl
            << "  cutoffAngle = " << (m_cutoffAngle * 180/M_PI) << std::endl
            << "  samplingWeight = " << m_samplingWeight << "," << endl
            << "  surfaceArea = ";
        if (m_shape)
            oss << m_shape->getSurfaceArea();
        else
            oss << "<no shape attached!>";
        oss << "," << endl
            << "  medium = " << indent(m_medium.toString()) << endl
            << "]";
        return oss.str();
    }

    Shader *createShader(Renderer *renderer) const;

    MTS_DECLARE_CLASS()
protected:
    Spectrum m_radiance, m_power;
    Float m_beamWidth, m_cutoffAngle;
    Float m_cosBeamWidth, m_cosCutoffAngle, m_invTransitionWidth;
};

// ================ Hardware shader implementation ================

class SpatialVaryingAreaShader : public Shader {
public:
    SpatialVaryingAreaShader(Renderer *renderer, const Spectrum &radiance,
        Float invTransitionWidth, Float cutoffAngle, Float cosCutoffAngle,
        Float cosBeamWidth)
        : Shader(renderer, EEmitterShader), m_radiance(radiance),
    	  m_invTransitionWidth(invTransitionWidth), m_cutoffAngle(cutoffAngle),
          m_cosCutoffAngle(cosCutoffAngle), m_cosBeamWidth(cosBeamWidth) {
    }

    void resolve(const GPUProgram *program, const std::string &evalName,
            std::vector<int> &parameterIDs) const {
        parameterIDs.push_back(program->getParameterID(evalName + "_radiance", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_invTransitionWidth", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_cutoffAngle", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_cosCutoffAngle", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_cosBeamWidth", false));
        
    }

    void generateCode(std::ostringstream &oss, const std::string &evalName,
            const std::vector<std::string> &depNames) const {
        oss << "uniform vec3 " << evalName << "_radiance;" << endl
            << "uniform float " << evalName << "_invTransitionWidth;" << endl
            << "uniform float " << evalName << "_cutoffAngle;" << endl
            << "uniform float " << evalName << "_cosCutoffAngle;" << endl
            << "uniform float " << evalName << "_cosBeamWidth;" << endl
            << endl
            << "vec3 " << evalName << "_area(vec2 uv) {" << endl
            << "    return " << evalName << "_radiance * pi;" << endl
            << "}" << endl
            << endl
            << "vec3 " << evalName << "_dir(vec3 wo) {" << endl
            << "    if (cosTheta(wo) < 0.0)" << endl
            << "        return vec3(0.0);" << endl
            << "    return vec3(inv_pi);" << endl
            << "}" << endl;
    }

    void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
        int &textureUnitOffset) const {
        program->setParameter(parameterIDs[0], m_radiance);
        program->setParameter(parameterIDs[1], m_invTransitionWidth);
        program->setParameter(parameterIDs[2], m_cutoffAngle);
        program->setParameter(parameterIDs[3], m_cosCutoffAngle);
        program->setParameter(parameterIDs[4], m_cosBeamWidth);
    }

    MTS_DECLARE_CLASS()
private:
    Spectrum m_radiance;
    Float m_invTransitionWidth;
    Float m_cutoffAngle, m_cosCutoffAngle;
    Float m_cosBeamWidth;
};

Shader *SpatialVaryingArea::createShader(Renderer *renderer) const {
    return new SpatialVaryingAreaShader(renderer, m_radiance,
    	m_invTransitionWidth, m_cutoffAngle, m_cosCutoffAngle,
        m_cosBeamWidth);
}

MTS_IMPLEMENT_CLASS(SpatialVaryingAreaShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(SpatialVaryingArea, false, Emitter)
MTS_EXPORT_PLUGIN(SpatialVaryingArea, "SVArea light");
MTS_NAMESPACE_END
