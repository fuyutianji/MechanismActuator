// Angular blockage history: Observe distinguishes a bounded contact plateau from
// slow directed travel; Reset cancels evidence. No engine state or sleep writes.
#pragma once

#include "CoreMinimal.h"

namespace MechanismAngularBlockage
{
    struct FSample
    {
        double Time = 0;
        double Angle = 0;
    };

    class FWindow
    {
    public:
        double GetMotionSign() const { return MotionSign; }

        void Reset()
        {
            Motion.Reset();
            Contact.Reset();
            ReferenceSpeed = 0;
            MotionSign = 0;
            LastTime = -1;
            LastAngle = 0;
            UnwrappedAngle = 0;
        }

        bool Observe(double Time, double Angle, bool bResistingContact,
            double ConfirmSeconds, double JitterDegrees, double SpeedLimit)
        {
            if (!FMath::IsFinite(Time) || !FMath::IsFinite(Angle)) { Reset(); return false; }
            if (LastTime >= 0 && Time <= LastTime) return false;
            // Pauses, hitches and discontinuous poses cannot establish a plateau.
            const double Dt = LastTime >= 0 ? Time - LastTime : 0;
            const double Delta = LastTime >= 0 ? FMath::FindDeltaAngleDegrees(LastAngle, Angle) : 0;
            if (Dt > 0.1 || FMath::Abs(Delta) > 20.0) Reset();
            if (LastTime < 0) UnwrappedAngle = Angle;
            else UnwrappedAngle += Delta;
            LastTime = Time;
            LastAngle = Angle;
            const FSample Sample{Time, UnwrappedAngle};
            Motion.Add(Sample);
            Trim(Motion, Time - 0.1);

            // Calibrate from actual resolved travel, never the configured maximum
            // alone. A mechanism blocked from rest falls back to the normal timeout.
            const double MotionDt = Time - Motion[0].Time;
            if (MotionDt >= 0.05)
            {
                const double Advance = Sample.Angle - Motion[0].Angle;
                double Path = 0;
                for (int32 I = 1; I < Motion.Num(); ++I) Path += FMath::Abs(Motion[I].Angle - Motion[I - 1].Angle);
                if (FMath::Abs(Advance) >= FMath::Max(0.02, JitterDegrees * 2)
                    && FMath::Abs(Advance) >= Path * 0.9)
                {
                    double Speed = FMath::Abs(Advance) / MotionDt;
                    if (SpeedLimit > 0) Speed = FMath::Min(Speed, SpeedLimit);
                    if (Speed > ReferenceSpeed)
                    {
                        ReferenceSpeed = Speed;
                        MotionSign = FMath::Sign(Advance);
                    }
                }
            }

            if (!bResistingContact) { Contact.Reset(); return false; }
            Contact.Add(Sample);
            Trim(Contact, Time - ConfirmSeconds);
            if (Contact.Num() < 4 || Time - Contact[0].Time < ConfirmSeconds) return false;
            const double Duration = Time - Contact[0].Time;
            const double ExpectedTravel = ReferenceSpeed * Duration;
            // Insufficient resolution at a very low speed is not evidence of a jam.
            if (MotionSign == 0 || ExpectedTravel < FMath::Max(0.02, JitterDegrees * 2)) return false;

            double MinAngle = Sample.Angle;
            double MaxAngle = Sample.Angle;
            double Path = 0;
            double FirstBest = -TNumericLimits<double>::Max();
            double SecondBest = FirstBest;
            const double MidTime = (Time + Contact[0].Time) * 0.5;
            for (int32 I = 0; I < Contact.Num(); ++I)
            {
                const auto& Item = Contact[I];
                MinAngle = FMath::Min(MinAngle, Item.Angle);
                MaxAngle = FMath::Max(MaxAngle, Item.Angle);
                if (I > 0) Path += FMath::Abs(Item.Angle - Contact[I - 1].Angle);
                double& Best = Item.Time <= MidTime ? FirstBest : SecondBest;
                Best = FMath::Max(Best, MotionSign * Item.Angle);
            }
            if (MaxAngle - MinAngle > JitterDegrees) return false;
            const double NetProgress = MotionSign * (Sample.Angle - Contact[0].Angle);
            const double AllowedProgress = FMath::Min(JitterDegrees * 0.05, ExpectedTravel * 0.01);
            // Directed creep, including travel below the jitter amplitude, vetoes
            // early sleep. Oscillation is measured by its envelope, not peak speed.
            if (NetProgress > AllowedProgress || SecondBest - FirstBest > AllowedProgress
                || (NetProgress > 1.e-9 && NetProgress >= Path * 0.5)) return false;
            return true;
        }

    private:
        static void Trim(TArray<FSample>& Samples, double Cutoff)
        {
            int32 Remove = 0;
            // Retain the sample immediately before the window's left boundary.
            while (Remove + 1 < Samples.Num() && Samples[Remove + 1].Time <= Cutoff) ++Remove;
            if (Remove > 0) Samples.RemoveAt(0, Remove, EAllowShrinking::No);
            if (Samples.Num() > 256) Samples.RemoveAt(0, Samples.Num() - 256, EAllowShrinking::No);
        }

        TArray<FSample> Motion;
        TArray<FSample> Contact;
        double ReferenceSpeed = 0;
        double MotionSign = 0;
        double LastTime = -1;
        double LastAngle = 0;
        double UnwrappedAngle = 0;
    };
}
